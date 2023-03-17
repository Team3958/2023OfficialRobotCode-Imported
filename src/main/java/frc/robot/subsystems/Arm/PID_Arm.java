// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.PIDSTuff.PIDController;

public class PID_Arm extends SubsystemBase {

  private final WPI_TalonFX ShoulderLeft = new WPI_TalonFX(Constants.shoulder1);
  private final WPI_TalonFX ShoulderRight = new WPI_TalonFX(Constants.shoulder2);
  private final WPI_TalonFX Extension = new WPI_TalonFX(Constants.extend);
  private final WPI_TalonFX Wrist = new WPI_TalonFX(Constants.wrist);

  private final PIDController ShoulderPID = new PIDController(Constants.ShoulderP, Constants.ShoulderI, Constants.ShoulderD);
  private final PIDController ExtensionPID = new PIDController(Constants.ExtendP, Constants.ExtendI, Constants.ExtendD);
  private final PIDController WristPID = new PIDController(Constants.WristP, Constants.WristI, Constants.WristD);

  private Double ShoulderL_E;
  private Double ShoulderR_E;
  private Double Extension_E;
  private Double Wrist_E;

  private boolean flag = false;

  private DoubleSupplier SGoalAngle;
  private DoubleSupplier GoalLength;
  private DoubleSupplier WGoalAngle;

  public AHRS m_gyro = new AHRS(Port.kMXP);

  /** Creates a new PID_Arm. */
  public PID_Arm() {

    motor_init(ShoulderLeft);
    motor_init(ShoulderRight);
    motor_init(Extension);
    motor_init(Wrist);

    ShoulderL_E = ShoulderLeft.getSelectedSensorPosition(); 
    ShoulderR_E = ShoulderRight.getSelectedSensorPosition(); 
    Extension_E = Extension.getSelectedSensorPosition();
    Wrist_E = Wrist.getSelectedSensorPosition();

    ShoulderPID.setTolerance(5);
    ExtensionPID.setTolerance(5);
    WristPID.setTolerance(5);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void motor_init(WPI_TalonFX motor){
    motor.configFactoryDefault();
    motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,0);
  }

  public void ShoulderRotateToAngle(Double goalAngle){
    ShoulderLeft.setVoltage(ShoulderPID.calculate(SLgetAngle(), goalAngle));
    ShoulderRight.setVoltage(ShoulderPID.calculate(SRgetAngle(), goalAngle));
  }

  public void ExtendToLength(Double goalLength){
    Extension.setVoltage(ExtensionPID.calculate(GetExtendLength(), goalLength));
  }

  public void WristRotateToAngle(Double goalAngle){
    Wrist.setVoltage(WristPID.calculate(WrgetAngle(), goalAngle));
  }

  private double SLgetAngle(){
    double motor_Rotation = (ShoulderLeft.getSelectedSensorPosition()-ShoulderL_E)/Constants.kEncoderTicksPerRev;
    double angle = motor_Rotation*360;
    return angle;
  }

  private double SRgetAngle(){
    double motor_Rotation = (ShoulderRight.getSelectedSensorPosition()-ShoulderR_E)/Constants.kEncoderTicksPerRev;
    double angle = motor_Rotation*360;
    return angle;
  }

  //Current iteration is if we start with the arm down at all times
  //But if we start with the arm fully extended at all times
  //Then it is same just with FullArmLength instead of startingLength, and it is subtraction instead of addition
  private double GetExtendLength(){
    double motor_Rotation = (Extension.getSelectedSensorPosition()-Extension_E)/Constants.kEncoderTicksPerRev;
    double length = Constants.startingLength + (motor_Rotation*(1/Constants.RevPerMeter));
    return length;
  }

  private double WrgetAngle(){
    double motor_Rotation = (Wrist.getSelectedSensorPosition()-Wrist_E)/Constants.kEncoderTicksPerRev;
    double angle = motor_Rotation*360;
    return angle;
  }

  public void EnableShoulderPIDControl(DoubleSupplier SGA){
    SGoalAngle = SGA;
    flag = true;
  }

  public void EnableExtensionPIDControl(DoubleSupplier GL){
    GoalLength = GL;
    flag = true;
  }

  public void EnableWristPIDControl(DoubleSupplier WGA){
    WGoalAngle = WGA;
    flag = true;
  }

  public void DisableShoulerPIDControl(){
    flag = false;
    ShoulderLeft.set(ControlMode.PercentOutput, 0);
    ShoulderRight.set(ControlMode.PercentOutput, 0);
  }

  public void DisableExtensionPIDControl(){
    flag = false;
    Extension.set(ControlMode.PercentOutput, 0);
  }

  public void DisableWristPIDControl(){
    flag = false;
    Wrist.set(ControlMode.PercentOutput, 0);
  }

  public boolean ShoulderatSetpoint(){
    return ShoulderPID.atSetpoint();
  }

  public boolean ExtenderatSetpoint(){
    return ExtensionPID.atSetpoint();
  }
  
  public boolean WristatSetpoint(){
    return WristPID.atSetpoint();
  }

  public void resetGyro(){
    m_gyro.reset();
  }

}
