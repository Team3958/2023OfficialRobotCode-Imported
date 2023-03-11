// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm;

public class ArmSwing extends CommandBase {
  private arm Arm;
  private XboxController xc;
  /** Creates a new ArmSwing. */
  public ArmSwing(arm a, XboxController x) {
    // Use addRequirements() here to declare subsystem dependencies.
    Arm = a;
    xc = x;
    addRequirements(Arm);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Arm.move_shoulder(xc.getLeftX()*0.4);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
