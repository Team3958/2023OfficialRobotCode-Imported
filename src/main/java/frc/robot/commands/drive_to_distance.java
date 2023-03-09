// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class drive_to_distance extends CommandBase {
  /** Creates a new drive_to_distance. */
  private DriveTrain dt;
  private double velocity;
  // left to right
  private double x_comp;
  // forward and backwards
  private double y_comp;
  private TrapezoidProfile.Constraints profile;
  private ProfiledPIDController pid_fl;
  private ProfiledPIDController pid_fr;
  private ProfiledPIDController pid_bl;
  private ProfiledPIDController pid_br;

  public drive_to_distance(DriveTrain d, double[] point, double v) {
    // Use addRequirements() here to declare subsystem dependencies.
    dt = d;
    velocity = v;
    profile = new TrapezoidProfile.Constraints(v, v/1.5);
    x_comp = point[0];
    y_comp = point[1];
    pid_fl = new ProfiledPIDController(Constants.fl_kP, Constants.fl_kI, Constants.fl_kD, profile);
    pid_fr = new ProfiledPIDController(Constants.fr_kP, Constants.fr_kI, Constants.fr_kD, profile);
    pid_bl = new ProfiledPIDController(Constants.bl_kP, Constants.bl_kI, Constants.bl_kD, profile);
    pid_br = new ProfiledPIDController(Constants.br_kP, Constants.br_kI, Constants.br_kD, profile);
    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // calculate motor speeds
    
    // flv = y+x
    //frv = y - x
    // blv = y-x
    // brv = y + x 

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
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
