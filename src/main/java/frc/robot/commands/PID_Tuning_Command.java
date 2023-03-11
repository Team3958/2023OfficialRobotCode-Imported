// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.Console;
import java.lang.invoke.ConstantBootstraps;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenixpro.controls.VoltageOut;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class PID_Tuning_Command extends CommandBase {
  /** Creates a new PID_Tuning_Command. */
  private DriveTrain dt;
  private ProfiledPIDController fl;
  private double some;
  private double p;
  private double i;
  private double d;
  
  public PID_Tuning_Command(DriveTrain d) {
    // Use addRequirements() here to declare subsystem dependencies.
    dt=d;
    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    fl = new ProfiledPIDController(Constants.fl_kP, Constants.fl_kI, Constants.fl_kD, new TrapezoidProfile.Constraints(4, 2));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //dt.drive_by_voltage(fl.calculate(0));
    //dt.get_fl_motor().set(ControlMode.Position, fl.calculate(3));
    dt.drive_by_pose(3);
    some = dt.get_fl_motor().getSelectedSensorPosition();
    SmartDashboard.putNumber("Motor Postion", some);
    //p = SmartDashboard.getNumber("desired P", Constants.fl_kP);
    //i = SmartDashboard.getNumber("desired I", Constants.fl_kI);
    //d = SmartDashboard.getNumber("desired P", Constants.fl_kD);
    //fl.setP(p);
    //fl.setI(i);
    //fl.setD(d);
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
