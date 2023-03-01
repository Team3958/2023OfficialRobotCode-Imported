// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class intake extends SubsystemBase {
  /** Creates a new intake. */
  TalonSRX motor1 = new TalonSRX(Constants.intake1);
  // adjust config
  TalonSRXConfiguration config1 = new TalonSRXConfiguration();
      ;

  public intake() {
    motorinit(motor1, config1);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  private final void motorinit(TalonSRX motor, TalonSRXConfiguration config){
    motor.setNeutralMode(NeutralMode.Coast);
    motor.configAllSettings(config);
  }
  public void Intaking(double speed){
    motor1.set(ControlMode.PercentOutput, speed);
  }
  public void Extaking(double speed){
    motor1.set(ControlMode.PercentOutput, speed);
  }
}
