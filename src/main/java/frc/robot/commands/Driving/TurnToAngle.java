// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Driving;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TurnToAngle extends CommandBase {
  public DriveTrain dt;
  public DoubleSupplier ga;
  public Double GoalAngle = 0.0;
  public Double CurrentAngle = 0.0;
  public Double StartingAngle = 0.0;

  /** Creates a new TurnToAngle. */
  public TurnToAngle(DriveTrain td, DoubleSupplier ag) {
    dt = td;
    ga = ag;

    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(td);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    GoalAngle = ga.getAsDouble();
    StartingAngle = dt.getAngle();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    CurrentAngle = dt.getAngle();
    double PercentError = (GoalAngle - (CurrentAngle- StartingAngle))/Math.abs(GoalAngle);
    boolean isBackwards = PercentError<0; 
    double absPercentError = Math.abs(PercentError); 
    

    
    double motorOutput = 0.45; 
    if(absPercentError<=1 && absPercentError > 0.8) {
      motorOutput = 0.75*(1-absPercentError) + 0.3; 
    } else if (absPercentError<0.25) {
      motorOutput = 0.9*absPercentError + 0.225; 
    }

    double lowMotorOutput = 0.35;
    if(GoalAngle <= 45) {
      if(absPercentError <=1 && absPercentError > 0.8)
      lowMotorOutput = 0.5*(1-absPercentError) + 0.25; 
        else if (absPercentError < 0.6) {
          lowMotorOutput = 0.1667*absPercentError + 0.25; 
        }
      } 
      
    motorOutput *= isBackwards ? -1 : 1;
    lowMotorOutput *= isBackwards ? -1 : 1;

    dt.telop_drive(0, 0, motorOutput);
    dt.telop_drive(0, 0, lowMotorOutput);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    dt.telop_drive(0, 0, 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double percentError =  (GoalAngle - (CurrentAngle- StartingAngle))/Math.abs(GoalAngle);
    return percentError > -.01 && percentError < .01;
  }
}