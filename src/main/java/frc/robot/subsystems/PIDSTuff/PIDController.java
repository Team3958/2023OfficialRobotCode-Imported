// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.PIDSTuff;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings("PMD.TooManyFields")
public class PIDController implements Sendable, AutoCloseable {
  private static int instances;

  // Factor for "proportional" control
  private double m_kp;

  // Factor for "integral" control
  private double m_ki;

  // Factor for "derivative" control
  private double m_kd;

  // The period (in seconds) of the loop that calls the controller
  private final double m_period;

  private double m_maximumIntegral = 1.0;

  private double m_minimumIntegral = -1.0;

  private double m_maximumInput;

  private double m_minimumInput;

  // Do the endpoints wrap around? eg. Absolute encoder
  private boolean m_continuous;

  // The error at the time of the most recent call to calculate()
  private double m_positionError;
  private double m_velocityError;

  // The error at the time of the second-most-recent call to calculate() (used to compute velocity)
  private double m_prevError;

  // The sum of the errors for use in the integral calc
  private double m_totalError;

  // The error that is considered at setpoint.
  private double m_positionTolerance = 0.05;
  private double m_velocityTolerance = Double.POSITIVE_INFINITY;

  private double m_setpoint;
  private double m_measurement;

  public PIDController(double kp, double ki, double kd) {
    this(kp, ki, kd, 0.02);
  }

  public PIDController(double kp, double ki, double kd, double period) {
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;

    if (period <= 0) {
      throw new IllegalArgumentException("Controller period must be a non-zero positive number!");
    }
    m_period = period;

    instances++;
    SendableRegistry.addLW(this, "PIDController", instances);

    HAL.report(tResourceType.kResourceType_PIDController2, instances);
  }

  @Override
  public void close() throws Exception {
    // TODO Auto-generated method stub
    SendableRegistry.remove(this);
  }

  public void setPID(double kp, double ki, double kd) {
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
  }

  public void setP(double kp) {
    m_kp = kp;
  }

  public void setI(double ki) {
    m_ki = ki;
  }

  public void setD(double kd) {
    m_kd = kd;
  }

  public double getP() {
    return m_kp;
  }

  public double getI() {
    return m_ki;
  }

  public double getD() {
    return m_kd;
  }

  public double getPeriod() {
    return m_period;
  }

  public void setSetpoint(double setpoint) {
    m_setpoint = setpoint;
  }

  public double getSetpoint() {
    return m_setpoint;
  }

  public boolean atSetpoint() {
    double positionError;
    if (m_continuous) {
        double errorBound = (m_maximumInput - m_minimumInput) / 2.0;
          positionError = MathUtil.inputModulus(m_setpoint - m_measurement, -errorBound, errorBound);
    } else {
      positionError = m_setpoint - m_measurement;
    }

    double velocityError = (positionError - m_prevError) / m_period;

    return Math.abs(positionError) < m_positionTolerance
        && Math.abs(velocityError) < m_velocityTolerance;
  }

  public void enableContinuousInput(double minimumInput, double maximumInput) {
    m_continuous = true;
    m_minimumInput = minimumInput;
    m_maximumInput = maximumInput;
  }

  public void disableContinuousInput() {
    m_continuous = false;
  }

  public boolean isContinuousInputEnabled() {
    return m_continuous;
  }

  public void setIntegratorRange(double minimumIntegral, double maximumIntegral) {
    m_minimumIntegral = minimumIntegral;
    m_maximumIntegral = maximumIntegral;
  }

  public void setTolerance(double positionTolerance) {
    setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
  }

  public void setTolerance(double positionTolerance, double velocityTolerance) {
    m_positionTolerance = positionTolerance;
    m_velocityTolerance = velocityTolerance;
  }

  public double getPositionError() {
    return m_positionError;
  }

  /** Returns the velocity error. */
  public double getVelocityError() {
    return m_velocityError;
  }

  public double calculate(double measurement, double setpoint) {
    // Set setpoint to provided value
    setSetpoint(setpoint);
    return calculate(measurement);
  }

  public double calculate(double measurement) {
    m_measurement = measurement;
    m_prevError = m_positionError;

    if (m_continuous) {
      m_positionError =
          MathUtil.inputModulus(m_setpoint - measurement, m_minimumInput, m_maximumInput);
    } else {
      m_positionError = m_setpoint - measurement;
    }

    m_velocityError = (m_positionError - m_prevError) / m_period;

    if (m_ki != 0) {
      m_totalError =
          MathUtil.clamp(
              m_totalError + m_positionError * m_period,
              m_minimumIntegral / m_ki,
              m_maximumIntegral / m_ki);
    }

    return m_kp * m_positionError + m_ki * m_totalError + m_kd * m_velocityError;
  }

  /** Resets the previous error and the integral term. */
  public void reset() {
    m_prevError = 0;
    m_totalError = 0;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub
    builder.setSmartDashboardType("PIDController");
    builder.addDoubleProperty("p", this::getP, this::setP);
    builder.addDoubleProperty("i", this::getI, this::setI);
    builder.addDoubleProperty("d", this::getD, this::setD);
    builder.addDoubleProperty("setpoint", this::getSetpoint, this::setSetpoint);
    
  }

}
