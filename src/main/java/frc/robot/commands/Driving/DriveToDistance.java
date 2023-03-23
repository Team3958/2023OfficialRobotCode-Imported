// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Driving;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveToDistance extends CommandBase {
  private DriveTrain dt;
  private double DistanceToTravel = 0.0;
  private final double angleTolerance = 2;
  private double dtt;
  private double StartX = 0.0;
  private double StartY = 0.0; 
  private double startAngle;

  /** Creates a new DriveToDistance. */
  public DriveToDistance(DriveTrain td, double ttd) {

    dt = td;
    dtt = ttd;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(td);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    StartX = dt.getCurrentX();
    StartY = dt.getCurrentY();
    DistanceToTravel = dtt;
    startAngle = dt.getAngle();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distanceTravelled = Math.sqrt(Math.pow(dt.getCurrentX()-StartX, 2) + Math.pow(dt.getCurrentY()-StartY, 2));

    double percentError = (Math.abs(DistanceToTravel)-distanceTravelled)/DistanceToTravel;
    boolean isBackwards = percentError<0; 
    double absPercentError = Math.abs(percentError); 

    double motorOutputL = 0.6;
    double motorOutputR = 0.6; 
    
    if(absPercentError<=1 && absPercentError > 0.8) {
      motorOutputL = 1*(1-absPercentError) + 0.4;
      motorOutputR = 1*(1-absPercentError) + 0.4; 
    } else if (absPercentError<0.25) {
      motorOutputL = 1.4*absPercentError + 0.25;
      motorOutputR = 1.4*absPercentError + 0.25; 
    }
    double angleOffset = dt.getAngle()- startAngle;
    
    if(angleOffset< angleTolerance ){
      motorOutputL *= 1.1;
    }
    else if (angleOffset > angleTolerance){
      motorOutputR *= 1.1;
    }


    motorOutputL *= isBackwards ? 1 : -1; 
    motorOutputR *= isBackwards ? 1 : -1; 

    MathUtil.clamp(motorOutputL, -1, 1);
    MathUtil.clamp(motorOutputR, -1, 1);
    System.out.println(motorOutputL);



    dt.tankish_drive(motorOutputL, motorOutputR);;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    dt.tankish_drive(0, 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double distanceTravelled = Math.sqrt(Math.pow(dt.getCurrentX()-StartX, 2) + Math.pow(dt.getCurrentY()-StartY, 2));
    return (Math.abs(DistanceToTravel) - distanceTravelled) < 0.005 || Math.abs(DistanceToTravel) < 0.1; //sets tolerence;
  }
}