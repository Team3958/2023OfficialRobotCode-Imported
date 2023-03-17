// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.PID_Arm;

public class RotateShoulderAuton extends CommandBase {

  private PID_Arm Arm;
  private double goalAngle;

    /** Creates a new RotateShoulderAuton. */
  public RotateShoulderAuton(PID_Arm A, double ga) {

    Arm = A;
    goalAngle = ga;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(A);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Arm.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Arm.ShoulderRotateToAngle(goalAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Arm.DisableShoulerPIDControl();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Arm.ShoulderatSetpoint() == true){
      Arm.DisableShoulerPIDControl();
      return true;
    }
    else {
      return false;
    }
  }
}
