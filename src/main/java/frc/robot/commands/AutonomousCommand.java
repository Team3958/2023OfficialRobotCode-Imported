// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

/** An example command that uses an example subsystem. */
public class AutonomousCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain dt;
  private int routineNumber;
  private double tick = Robot.tick; 
  /*
    ^^ goofy wacky stuff going on here. tick increments every time auton periodic is called in Robot.
    That value is used to calcluate what should be the speeds needed to drive the robot along the curve
    defined in path(), x(), and y().
  */
  /**
   * Creates a new ExampleCommand.
   *
  @param d The subsystem used by this command.
   */
  public AutonomousCommand(DriveTrain d) {
    dt = d;
    routineNumber = Constants.AutonomousRoutine;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(d);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tick = Robot.tick;
    if (0 < tick && tick < 150) { // 150 is a guess, will be however many ticks to complete block placement
      ; // This is where I will call the method to place the block
    } else if (151 < tick && tick < 450) { // 450 also a guess
      // driving to the middle of the field
      double[] fromPath = path();
      double x = fromPath[0]; // gotta swap x and y?
      double y = fromPath[1];
      dt.MechDrive(x, y, 0, dt.geRotation2d()); // drive along the path
    } else { // rest of auton period is spent driving onto platform
      ; // Method to drive onto platform
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.MechDrive(0, 0, 0, null);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  /*
   * path() is used to describe different bezier curves
   * the general equation of the curve is defined in x() and y()
   */
  private double[] path() {
    double[] returns = new double[2];
    switch(routineNumber) {
      case 1: returns[0] = x(0, 650, 710, tick + 1.0/350.0) - x(0, 650, 710, tick); // "dx" component for routine 1
              returns[1] = y(445, 600, 275, tick + 1.0/350.0) - y(445, 600, 275, tick); // "dy" component for routine 1
      case 2: returns[0] = 0; // "dx" component for routine 2
              returns[1] = 0; // "dy" component for routine 2
      // gotta find a new curve for case 3 atm
      case 3: returns[0] = x(0, 548, 548, tick + 1.0/350.0) - x(0, 548, 548, tick); // "dx" component for routine 3
              returns[1] = y(105, 52, 275, tick + 1.0/350.0) - y(105, 52, 275, tick); // "dy" component for routine 3
    }
    // This is trying to use the step from one call of tick to the next, and unitize it
    double magnitude = Math.sqrt(Math.pow(returns[0], 2) + Math.pow(returns[1], 2));

    returns[0] = returns[0] / magnitude;
    returns[1] = returns[1] / magnitude;

    return returns;
  }
  private double x(int x0, int x1, int x2, double tick) {
    if (tick < 350) {
      tick = tick - 150 / 350.0; // 150 is the same guess made in execute(), subject to change
      return Math.pow((1 - tick), 2) * x0 + 2 * (1 - tick) * tick * x1 + Math.pow(tick, 2) * x2;
    }
    return 0;
    
  }
  private double y(int y0, int y1, int y2, double tick) {
    if (tick < 350) {
      tick = tick - 150 / 350.0; // 150 is the same guess made in execute(), subject to change
      return Math.pow((1 - tick), 2) * y0 + 2 * (1 - tick) * tick * y1 + Math.pow(tick, 2) * y2;
    }
    return 0;
  }
}
