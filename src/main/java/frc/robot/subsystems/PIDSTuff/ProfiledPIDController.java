// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.PIDSTuff;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ProfiledPIDController implements Sendable {
  private static int instances;

  private PIDController m_controller;
  private double m_minimumInput;
  private double m_maximumInput;
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
  private TrapezoidProfile.Constraints m_constraints;

  @SuppressWarnings("ParameterName")
  public ProfiledPIDController(
      double Kp, double Ki, double Kd, TrapezoidProfile.Constraints constraints) {
    this(Kp, Ki, Kd, constraints, 0.02);
  }

  @SuppressWarnings("ParameterName")
  public ProfiledPIDController(
      double Kp, double Ki, double Kd, TrapezoidProfile.Constraints constraints, double period) {
    m_controller = new PIDController(Kp, Ki, Kd, period);
    m_constraints = constraints;
    instances++;
    HAL.report(tResourceType.kResourceType_ProfiledPIDController, instances);
  }

  @SuppressWarnings("ParameterName")
  public void setPID(double Kp, double Ki, double Kd) {
    m_controller.setPID(Kp, Ki, Kd);
  }

  @SuppressWarnings("ParameterName")
  public void setP(double Kp) {
    m_controller.setP(Kp);
  }

  @SuppressWarnings("ParameterName")
  public void setI(double Ki) {
    m_controller.setI(Ki);
  }


  @SuppressWarnings("ParameterName")
  public void setD(double Kd) {
    m_controller.setD(Kd);
  }

  public double getP() {
    return m_controller.getP();
  }

  public double getI() {
    return m_controller.getI();
  }

  public double getD() {
    return m_controller.getD();
  }

  public double getPeriod() {
    return m_controller.getPeriod();
  }

  public void setGoal(TrapezoidProfile.State goal) {
    m_goal = goal;
  }

  public void setGoal(double goal) {
    m_goal = new TrapezoidProfile.State(goal, 0);
  }

  public TrapezoidProfile.State getGoal() {
    return m_goal;
  }

  public boolean atGoal() {
    return atSetpoint() && m_goal.equals(m_setpoint);
  }

  public void setConstraints(TrapezoidProfile.Constraints constraints) {
    m_constraints = constraints;
  }

  public TrapezoidProfile.State getSetpoint() {
    return m_setpoint;
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  public void enableContinuousInput(double minimumInput, double maximumInput) {
    m_controller.enableContinuousInput(minimumInput, maximumInput);
    m_minimumInput = minimumInput;
    m_maximumInput = maximumInput;
  }

  /** Disables continuous input. */
  public void disableContinuousInput() {
    m_controller.disableContinuousInput();
  }

  public void setIntegratorRange(double minimumIntegral, double maximumIntegral) {
    m_controller.setIntegratorRange(minimumIntegral, maximumIntegral);
  }

  public void setTolerance(double positionTolerance) {
    setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
  }

  public void setTolerance(double positionTolerance, double velocityTolerance) {
    m_controller.setTolerance(positionTolerance, velocityTolerance);
  }

  public double getPositionError() {
    return m_controller.getPositionError();
  }

  public double getVelocityError() {
    return m_controller.getVelocityError();
  }

  public double calculate(double measurement) {
    if (m_controller.isContinuousInputEnabled()) {
      // Get error which is smallest distance between goal and measurement
      double errorBound = (m_maximumInput - m_minimumInput) / 2.0;
      double goalMinDistance =
        MathUtil.inputModulus(m_goal.position - measurement, -errorBound, errorBound);
      double setpointMinDistance =
        MathUtil.inputModulus(m_setpoint.position - measurement, -errorBound, errorBound);

      // Recompute the profile goal with the smallest error, thus giving the shortest path. The goal
      // may be outside the input range after this operation, but that's OK because the controller
      // will still go there and report an error of zero. In other words, the setpoint only needs to
      // be offset from the measurement by the input range modulus; they don't need to be equal.
      m_goal.position = goalMinDistance + measurement;
      m_setpoint.position = setpointMinDistance + measurement;
    }

    var profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);
    m_setpoint = profile.calculate(getPeriod());
    return m_controller.calculate(measurement, m_setpoint.position);
  }

  public double calculate(double measurement, TrapezoidProfile.State goal) {
    setGoal(goal);
    return calculate(measurement);
  }

  public double calculate(double measurement, double goal) {
    setGoal(goal);
    return calculate(measurement);
  }

  public double calculate(
      double measurement, TrapezoidProfile.State goal, TrapezoidProfile.Constraints constraints) {
    setConstraints(constraints);
    return calculate(measurement, goal);
  }

  public void reset(TrapezoidProfile.State measurement) {
    m_controller.reset();
    m_setpoint = measurement;
  }

  public void reset(double measuredPosition, double measuredVelocity) {
    reset(new TrapezoidProfile.State(measuredPosition, measuredVelocity));
  }

  public void reset(double measuredPosition) {
    reset(measuredPosition, 0.0);
  }


  public ProfiledPIDController() {}

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("ProfiledPIDController");
    builder.addDoubleProperty("p", this::getP, this::setP);
    builder.addDoubleProperty("i", this::getI, this::setI);
    builder.addDoubleProperty("d", this::getD, this::setD);
    builder.addDoubleProperty("goal", () -> getGoal().position, this::setGoal);
  }
  
}
