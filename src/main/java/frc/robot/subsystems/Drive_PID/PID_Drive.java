// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive_PID;

import java.util.function.DoubleSupplier;

import javax.swing.text.StyledEditorKit.BoldAction;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PID_Drive extends SubsystemBase {
  /** Creates a new PID_Drive. */
  //create motors 
  private final WPI_TalonFX frontL = new WPI_TalonFX(Constants.frontleft);
  private final WPI_TalonFX frontR = new WPI_TalonFX(Constants.frontright);
  private final WPI_TalonFX backL = new WPI_TalonFX(Constants.backleft);
  private final WPI_TalonFX backR = new WPI_TalonFX(Constants.backright);

  //Create PIDs
  private final PIDController fl_Controller = new PIDController(Constants.fl_kP, Constants.fl_kI,Constants.fl_kD);
  private final PIDController fr_Controller = new PIDController(Constants.fr_kP, Constants.fr_kI,Constants.fr_kD);
  private final PIDController bl_Controller = new PIDController(Constants.bl_kP, Constants.fl_kI,Constants.bl_kD);
  private final PIDController br_Controller = new PIDController(Constants.br_kP, Constants.br_kI,Constants.br_kD);

  private final PIDController anglePID = new PIDController(Constants.turn_kP, Constants.turn_kI,Constants.turn_kD);

  // initial encoder position
  private double fl_E;
  private double fr_E;
  private double bl_E;
  private double br_E;

  // drive command flag
  private boolean flag = false;

  //motor goals
  private DoubleSupplier fl_goal;
  private DoubleSupplier fr_goal;
  private DoubleSupplier bl_goal;
  private DoubleSupplier br_goal;

  // create gyro
  public AHRS m_gyro = new AHRS(Port.kMXP);

  public PID_Drive() {
    motor_init(frontL);
    motor_init(frontR);
    motor_init(backR);
    motor_init(backL);

    fl_E = frontL.getSelectedSensorPosition();
    fr_E = frontR.getSelectedSensorPosition();
    bl_E = backL.getSelectedSensorPosition();
    br_E = backR.getSelectedSensorPosition();

    backL.setInverted(InvertType.InvertMotorOutput);
    frontL.setInverted(InvertType.InvertMotorOutput);

    //anglePID.enableContinuousInput(0, 360);
    // angle tolerence is 2 degrees
    anglePID.setTolerance(5);

    // distance tolerence is 5cm
    fl_Controller.setTolerance(0.05);
    fr_Controller.setTolerance(0.05);
    bl_Controller.setTolerance(0.05);
    br_Controller.setTolerance(0.05);
    
  }
  double x;
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    /*if (flag == true){
      frontL.setVoltage(fl_Controller.calculate(get_FL_distance(), fl_goal.getAsDouble()));
      frontR.setVoltage(fr_Controller.calculate(get_FR_distance(), fr_goal.getAsDouble()));
      backL.setVoltage(bl_Controller.calculate(get_BL_distance(), bl_goal.getAsDouble()));
      backR.setVoltage(br_Controller.calculate(get_BR_distance(), br_goal.getAsDouble()));
    }*/
    SmartDashboard.putNumber("encoder", frontL.getSelectedSensorPosition());
  }
  private void motor_init(TalonFX motor){
    motor.configFactoryDefault();
    motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,0);
  }

  public void run(double fl,  double fr, double bl, double br){
    frontL.setVoltage(fl_Controller.calculate(get_FL_distance(), fl));
    frontR.setVoltage(fr_Controller.calculate(get_FR_distance(), fr));
    backL.setVoltage(bl_Controller.calculate(get_BL_distance(), bl));
    backR.setVoltage(br_Controller.calculate(get_BR_distance(), br));
  }

  public void turnRun(double angle){
    frontL.setVoltage(anglePID.calculate(getHeading(), angle));
    backL.setVoltage(anglePID.calculate(getHeading(),angle));
    frontR.setVoltage(-anglePID.calculate(getHeading(), angle));
    backR.setVoltage(-anglePID.calculate(getHeading(), angle));
  }

  public double get_FL_distance(){
    double motor_rotation = (frontL.getSelectedSensorPosition()- fl_E)/Constants.kEncoderTicksPerRev;
    double wheel_rotaion = motor_rotation* Constants.kGearRatio;
    double distance = 2* Math.PI* Constants.kWheelRadiusMeters * wheel_rotaion;
    return distance;
  }

  public void rest_fl_encoder(){
    fl_E = frontL.getSelectedSensorPosition();
  }

  public double get_FR_distance(){
    double motor_rotation = (-frontR.getSelectedSensorPosition() - fr_E)/Constants.kEncoderTicksPerRev;
    double wheel_rotaion = motor_rotation* Constants.kGearRatio;
    double distance = 2* Math.PI* Constants.kWheelRadiusMeters * wheel_rotaion;
    return distance;
  }

  public void rest_fr_encoder(){
    fr_E = frontR.getSelectedSensorPosition();
  }

  public double get_BL_distance(){
    double motor_rotation = (backL.getSelectedSensorPosition() - bl_E)/Constants.kEncoderTicksPerRev;
    double wheel_rotaion = motor_rotation* Constants.kGearRatio;
    double distance = 2* Math.PI* Constants.kWheelRadiusMeters * wheel_rotaion;
    return distance;
  }

  public void rest_bl_encoder(){
    bl_E = backL.getSelectedSensorPosition();
  }

  public double get_BR_distance(){
    double motor_rotation = (backR.getSelectedSensorPosition() - br_E)/Constants.kEncoderTicksPerRev;
    double wheel_rotaion = motor_rotation* Constants.kGearRatio;
    double distance = 2* Math.PI* Constants.kWheelRadiusMeters * wheel_rotaion;
    return distance;
  }

  public void rest_br_encoder(){
    br_E = backR.getSelectedSensorPosition();
  }

  public void EnablePIDControll(DoubleSupplier fl, DoubleSupplier fr, DoubleSupplier bl, DoubleSupplier br){
    fl_goal = fl;
    fr_goal = fr;
    bl_goal = bl;
    br_goal = br;
    flag = true;
  }
  public void DisablePIDControll(){
    flag = false;
    frontL.set(ControlMode.PercentOutput, 0);
    frontR.set(ControlMode.PercentOutput, 0);
    backL.set(ControlMode.PercentOutput, 0);
    backR.set(ControlMode.PercentOutput, 0);
  }

  public boolean atSetpointFL(){
    return fl_Controller.atSetpoint();
  }
  public boolean atSetpointFR(){
    return fr_Controller.atSetpoint();
  }
  public boolean atSetpointBL(){
    return bl_Controller.atSetpoint();
  }
  public boolean atSetpointBR(){
    return br_Controller.atSetpoint();
  }
  public boolean atSetpointAngle(){
    return anglePID.atSetpoint();
  }

  public double getHeading(){
    return m_gyro.getAngle();
  }

  public void resetGyro(double offset){
    m_gyro.reset();
    m_gyro.setAngleAdjustment(offset);
  }

  public void restAnglePID(){
    anglePID.reset();
  }


}
