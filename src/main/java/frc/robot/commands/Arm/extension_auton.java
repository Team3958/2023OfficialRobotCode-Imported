// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm;

public class extension_auton extends CommandBase {
  /** Creates a new extension_auton. */
  private arm arm;
  private double start_extend;
  private final double goal = 25;
  private final double tolerance = 0.10;
  private double error;


  public extension_auton(arm ar) {
    // Use addRequirements() here to declare subsystem dependencies.
    arm  = ar;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    start_extend =  arm.get_extend_encoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double current_extend =  arm.extension_ticks_to_inches(arm.get_extend_encoder());
    error = (goal - current_extend)/goal;

    MathUtil.clamp(error,-.45 ,0.45);

    arm.move_extend(error);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.move_extend(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (error < tolerance){
      return true;
    }
    return false;
  }
}
