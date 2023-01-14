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
  private int tick = Robot.tick; 
  /*
    ^^ goofy wacky stuff going on here. tick increments every time auton periodic is called in Robot.
    That value is used to calcluate what should be the speeds needed to drive the robot along the curve
    defined in path().
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

    switch(routineNumber){
      case 1: // driving commands to be figured out
      case 2: // driving commands to be figured out
      case 3: // driving commands to be figured out
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
   * path is a parametric function that describes a curve, the path the robot wants to move on the field
   * feed it 
   */
  private int[] path() {
    int t = tick;
    int[] returns = new int[2];
    switch(routineNumber) {
      case 1: returns[0] = 0; // x component for routine 1
              returns[1] = 0; // y component for routine 1
      case 2: returns[0] = 0; // x component for routine 2
              returns[1] = 0; // y component for routine 2
      case 3: returns[0] = 0; // x component for routine 3
              returns[1] = 0; // y component for routine 3
    }
    return returns;
  }
}
