// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive_PID.PID_Drive;

public class Final_Auton_Drive extends CommandBase {
  /** Creates a new Final_Auton_Drive. */
  private PID_Drive dt;
  private double x;
  private double y;
  private double speed;
  public Final_Auton_Drive(PID_Drive d, double X, double Y, double Speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    dt = d;
    x= X;
    y = Y;
    speed = Speed;
    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double xt = x/speed;
    double yt = y/speed;


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dt.run(y, y, y, y);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(dt.atSetpointFL() == true && dt.atSetpointFR() == true && dt.atSetpointBL() == true && dt.atSetpointBR() == true){
      dt.DisablePIDControll();
      return true;
    }
    else{
      return false;
    }
  }
}
