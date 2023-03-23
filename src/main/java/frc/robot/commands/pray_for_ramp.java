// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class pray_for_ramp extends CommandBase {
  /** Creates a new pray_for_ramp. */
  private DriveTrain dt;
  private boolean flag_start = true;
  private boolean flag_on_ramp = false;
  // degrees of tolerance
  private double tolerance = 2;
  //goal
  private final double dropped_ramp_angle = 15;

  private double error;
  // as in the p term in a pid
  private final double p = 0.5;

  public pray_for_ramp(DriveTrain d) {
    // Use addRequirements() here to declare subsystem dependencies.
    dt =d;
    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dt.zero();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (flag_on_ramp == true){
      error = dt.getPitch() / dropped_ramp_angle;
      //dt.drive_by_percent(p*error);
    }

    else if (Math.abs(dt.getPitch()) > tolerance){
      flag_on_ramp = true;
      flag_start = false;
    }
    else if (Math.abs(dt.getPitch()) <  tolerance){
      flag_on_ramp = false;
    }
    if (flag_start == true){
      //dt.drive_by_percent(p);
    }

  };

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //dt.drive_by_percent(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
