// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class last_try_auton extends CommandBase {
  /** Creates a new last_try_auton. */
  private DriveTrain dt;
  private Timer watch;
  public last_try_auton(DriveTrain d) {
    // Use addRequirements() here to declare subsystem dependencies.
    dt = d;
    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    watch = new Timer();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (watch.get() < 1){
      dt.telop_drive(0, 0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.telop_drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (watch.get()>4){
      return true;
    }
    
    else{
      return false;
    }
  }
}
