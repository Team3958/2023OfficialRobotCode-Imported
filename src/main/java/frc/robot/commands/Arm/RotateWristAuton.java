// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.PID_Arm;

public class RotateWristAuton extends CommandBase {
  PID_Arm Arm;
  double goalAngle;

  /** Creates a new RotateWristAuton. */
  public RotateWristAuton(PID_Arm A, double ga) {

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
    Arm.WristRotateToAngle(goalAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Arm.DisableWristPIDControl();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Arm.WristatSetpoint() == true){
      Arm.DisableWristPIDControl();
      return true;
    }
    else {
      return false;
    }
  }
}
