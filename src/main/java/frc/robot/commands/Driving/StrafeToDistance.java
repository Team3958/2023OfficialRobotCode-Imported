// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Driving;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class StrafeToDistance extends CommandBase {
  private DriveTrain dt;
  private Double DistanceToTravel = 0.0;
  private DoubleSupplier dtt;
  private Double StartX = 0.0;
  private Double StartY = 0.0; 

  /** Creates a new StrafeToDistance. */
  public StrafeToDistance(DriveTrain td, DoubleSupplier ttd) {

    dt = td;
    dtt = ttd;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    StartX = dt.getCurrentX();
    StartY = dt.getCurrentY();
    DistanceToTravel = dtt.getAsDouble();
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distanceTravelled = Math.sqrt(Math.pow(dt.getCurrentX()-StartX, 2) + Math.pow(dt.getCurrentY()-StartY, 2));

    double percentError = (Math.abs(DistanceToTravel)-distanceTravelled)/DistanceToTravel;
    boolean isBackwards = percentError<0; 
    double absPercentError = Math.abs(percentError); 

    double motorOutput = 0.6; 
    
    if(absPercentError<=1 && absPercentError > 0.8) {
      motorOutput = 1*(1-absPercentError) + 0.4; 
    } else if (absPercentError<0.25) {
      motorOutput = 1.4*absPercentError + 0.25; 
    }

    motorOutput *= isBackwards ? 1 : -1; 

    dt.telop_drive(0, motorOutput, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    dt.telop_drive(0, 0, 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double distanceTravelled = Math.sqrt(Math.pow(dt.getCurrentX()-StartX, 2) + Math.pow(dt.getCurrentY()-StartY, 2));
    return (Math.abs(DistanceToTravel) - distanceTravelled) < 0.005 || Math.abs(DistanceToTravel) < 0.1;
  }
}
