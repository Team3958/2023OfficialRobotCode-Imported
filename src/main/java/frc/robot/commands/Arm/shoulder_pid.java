// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm;

public class shoulder_pid extends CommandBase {
  /** Creates a new shoulder_pid. */
  private arm arm;
  private PIDController pid = new PIDController(0.2, 0, 0.0);
  public shoulder_pid(arm a) {
    // Use addRequirements() here to declare subsystem dependencies.
    arm = a;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = pid.calculate(arm.get_shoulder1_encoder(), arm.rads_to_ticks(Math.PI/6));
    // tune maxs
    //MathUtil.clamp(output, -0.2, 0.2);
    //arm.move_shoulder(output);
    arm.move_by_magic(output, arm.rads_to_ticks(Math.PI/6));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.move_shoulder(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
