// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Driving;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class drive_by_encoder extends CommandBase {
  /** Creates a new drive_by_encoder. */
  private DriveTrain dt;
  private double dtt;
  private double fl_start;
  private double fr_start;
  private double bl_start;
  private double br_start;

  
  private double fl_error;
  private double fr_error;
  private double bl_error;
  private double br_error;

  private double fl_ticks_traveled;

  private double tolerence = 0.05;
  public drive_by_encoder(DriveTrain d, double dis) {
    // Use addRequirements() here to declare subsystem dependencies.
    dt = d;
    dtt= dis;
    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    fl_start = dt.get_fl_encoder();
    fr_start = dt.get_fr_encoder();
    bl_start = dt.get_bl_encoder();
    br_start = dt.get_br_encoder();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    fl_ticks_traveled = dt.get_fl_encoder()-fl_start;
    double fr_ticks_traveled = dt.get_fr_encoder()-fr_start;
    double bl_ticks_traveled = dt.get_bl_encoder()-bl_start;
    double br_ticks_traveled = dt.get_br_encoder()-br_start;

    fl_error = (dtt-dt.tick_to_distance(fl_ticks_traveled))/dtt;
    fr_error = (dtt-dt.tick_to_distance(fr_ticks_traveled))/dtt;
    bl_error = (dtt-dt.tick_to_distance(bl_ticks_traveled))/dtt;
    br_error = (dtt-dt.tick_to_distance(br_ticks_traveled))/dtt;

    MathUtil.clamp(fl_error, -0.6, 0.6);
    MathUtil.clamp(fr_error, -0.6, 0.6);
    MathUtil.clamp(bl_error, -0.6, 0.6);
    MathUtil.clamp(br_error, -0.6, 0.6);

    dt.drive_by_percent(fl_error, fr_error, bl_error, br_error);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.drive_by_percent(0,0,0,0);
  }
    
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(fl_error) < tolerence && Math.abs(fr_error) < tolerence && Math.abs(bl_error) < tolerence && Math.abs(br_error) < tolerence){
      return true;
    }
    else if (dt.tick_to_distance(fl_ticks_traveled) >= dtt){
      return true;
    }
    else{
      return false;
    }
  }
}
