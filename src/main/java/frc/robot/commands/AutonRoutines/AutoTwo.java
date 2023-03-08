// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonRoutines;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.commands.Auton.MPController;

public class AutoTwo extends CommandBase {
  MPController mpc;
  
  /** Creates a new AutoTwo. */
  public AutoTwo() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*while (Robot.auto_done == false){
      mpc.drive.putEncoder();
      mpc.drive.putGyro();
      SmartDashboard.putNumber("X Pose (Ft): ", Units.metersToFeet(mpc.drive.getPose().getX()));
      SmartDashboard.putNumber("Y Pose (Ft): ", Units.metersToFeet(mpc.drive.getPose().getY()));
      SmartDashboard.putNumber("Rotation Pose (Degrees): ", mpc.drive.getPose().getRotation().getDegrees());
      mpc.drive.putWheelVelocities();
    }*/
    
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
