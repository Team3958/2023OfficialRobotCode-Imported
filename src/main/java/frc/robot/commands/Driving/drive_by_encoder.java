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

  private final double angleTolerance = 2;
  private double startAngle;
  private double direction;

  private double tolerence = 0.07;
  public drive_by_encoder(DriveTrain d, double dis) {
    // Use addRequirements() here to declare subsystem dependencies.
    dt = d;
    dtt= dis;
    direction = dis/Math.abs(dis);
    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    fl_start = dt.get_fl_encoder();
    fr_start = dt.get_fr_encoder();
    bl_start = dt.get_bl_encoder();
    br_start = dt.get_br_encoder();
    startAngle = dt.getAngle();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double fl_ticks_traveled = dt.get_fl_encoder()-fl_start;
    double fr_ticks_traveled = dt.get_fr_encoder()-fr_start;
    double bl_ticks_traveled = dt.get_bl_encoder()-bl_start;
    double br_ticks_traveled = dt.get_br_encoder()-br_start;

    fl_error = ((dtt-dt.tick_to_distance(fl_ticks_traveled))/dtt);
    fr_error = ((dtt-dt.tick_to_distance(fr_ticks_traveled))/dtt);
    bl_error = ((dtt-dt.tick_to_distance(bl_ticks_traveled))/dtt);
    br_error = ((dtt-dt.tick_to_distance(br_ticks_traveled))/dtt);

    double flmotorOutput = 0.6;
    double blmotorOutput = 0.6;
    double frmotorOutput = 0.6;
    double brmotorOutput = 0.6;

    if(fl_error<=1 && fl_error > 0.8) {
      flmotorOutput = 1*(1-fl_error) + 0.4; 
    } else if (fl_error<0.25) {
      flmotorOutput = 1.4*fl_error + 0.25; 
    }

    if(bl_error<=1 && bl_error > 0.8) {
      blmotorOutput = 1*(1-bl_error) + 0.4; 
    } else if (bl_error<0.25) {
      blmotorOutput = 1.4*bl_error + 0.25; 
    }

    if(fr_error<=1 && fr_error > 0.8) {
      frmotorOutput = 1*(1-fr_error) + 0.4; 
    } else if (fr_error<0.25) {
      frmotorOutput = 1.4*fr_error + 0.25; 
    }

    if(br_error<=1 && br_error > 0.8) {
      brmotorOutput = 1*(1-br_error) + 0.4; 
    } else if (br_error<0.25) {
      brmotorOutput = 1.4*br_error + 0.25; 
    }

    double angleOffset = dt.getAngle()- startAngle;
    
    if(angleOffset > angleTolerance ) {
      flmotorOutput *= 1.1;
      blmotorOutput *= 1.1;
    }
    else if (angleOffset < angleTolerance) {
      frmotorOutput *= 1.1;
      brmotorOutput *= 1.1;
    }


    MathUtil.clamp(flmotorOutput, -0.6, 0.65);
    MathUtil.clamp(frmotorOutput, -0.6, 0.65);
    MathUtil.clamp(blmotorOutput, -0.6, 0.65);
    MathUtil.clamp(brmotorOutput, -0.6, 0.65);

    dt.drive_by_percent(flmotorOutput, frmotorOutput, blmotorOutput, brmotorOutput);
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
   /*  else if (dt.tick_to_distance(fl_ticks_traveled) >= dtt){
      return true;
    }*/
    else{
      return false;
    }
  }
}
