// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive_PID.PID_Drive;

public class turn_to_angle extends CommandBase {
  /** Creates a new turn_to_angle. */
  private PID_Drive dt;
  private double desiredAngle;
  public turn_to_angle(PID_Drive d, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    dt = d;
    desiredAngle = angle;
    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dt.restAnglePID();
    dt.resetGyro(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dt.turnRun(desiredAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.DisablePIDControll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (dt.atSetpointAngle() == true){
      dt.DisablePIDControll();
      return true;
    }
    else{
      return false;
    }
  }
}
