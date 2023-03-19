// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class arm extends SubsystemBase {
  private final WPI_TalonFX shoulder_motor = new WPI_TalonFX(Constants.shoulder1);
  private final WPI_TalonFX shoulder2_motor = new WPI_TalonFX(Constants.shoulder2);

  private final WPI_TalonFX extend_motor = new WPI_TalonFX(Constants.extend);

  private final WPI_TalonFX wrist_motor = new WPI_TalonFX(Constants.wrist);

  private final WPI_TalonSRX intake1_motor = new WPI_TalonSRX(Constants.intake1);
  private final WPI_TalonSRX intake2_motor = new WPI_TalonSRX(Constants.intake2);

  private Translation3d start = new Translation3d(0.5,0.5,0.5);
  private Rotation3d startAngle = new Rotation3d();
  private Pose3d posehand = new Pose3d(start,startAngle);


  /** Creates a new arm. */
  public arm() {
    TalonSRXConfiguration config1 = new TalonSRXConfiguration();
      config1.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder ;
      //init_arm_motor(shoulder_motor, config1);
      init_arm_motor(shoulder2_motor);
      shoulder2_motor.follow(shoulder_motor);
      shoulder2_motor.setInverted(true);

      init_arm_motor(extend_motor);

      init_arm_motor(wrist_motor);
      wrist_motor.setNeutralMode(NeutralMode.Coast);

      //init_arm_motor(intake1_motor);
      //init_arm_motor(intake2_motor);
      intake1_motor.configFactoryDefault();
      intake2_motor.configFactoryDefault();
      
      intake2_motor.follow(intake1_motor);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void init_arm_motor( WPI_TalonFX motor){
    motor.configFactoryDefault();
    motor.setNeutralMode(NeutralMode.Brake);
    
  }
  public void move_shoulder(double direction){
    shoulder_motor.set(direction);
  }

  public void move_extend(double direction){
    extend_motor.set(direction);
  }
  public void move_wrist(double direction){
    wrist_motor.set(direction);
  }
  public void move_grip(double direction){
    intake1_motor.set(direction);
  }
  public void rest_motor(){
    shoulder_motor.set(0);
    extend_motor.set(0);
    wrist_motor.set(0);
    intake1_motor.set(0);
  }
}
