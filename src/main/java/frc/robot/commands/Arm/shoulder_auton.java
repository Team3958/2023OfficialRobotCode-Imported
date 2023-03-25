// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.arm;

public class shoulder_auton extends CommandBase {
  /** Creates a new shoulder_auton. */
  private arm arm;
  private double error;
  private double tolerence = 0.07;

  public shoulder_auton(arm a) {
    // Use addRequirements() here to declare subsystem dependencies.
    arm =a;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = (Math.PI/3)- (arm.tick_to_rads(arm.get_shoulder1_encoder()))/ (Math.PI/3);
    MathUtil.clamp(error, -4, 0.4);
    arm.move_shoulder(error);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.move_shoulder(Constants.G_conpensate);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (error< tolerence){
      return true;
    }
    return false;
  }
}
