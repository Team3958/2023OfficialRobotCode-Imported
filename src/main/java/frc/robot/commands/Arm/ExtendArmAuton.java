// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.PID_Arm;

public class ExtendArmAuton extends CommandBase {
  PID_Arm Arm;
  double goalLength;

  /** Creates a new ExtendArmAuton. */
  public ExtendArmAuton(PID_Arm A, double gl) {

    Arm = A;
    goalLength = gl;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(A);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Arm.ExtendToLength(goalLength);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Arm.DisableExtensionPIDControl();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Arm.ExtenderatSetpoint() == true){
      Arm.DisableExtensionPIDControl();
      return true;
    }
    else {
      return false;
    }
  }
}
