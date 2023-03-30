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
  private double startTicks;
  private final double max = 0.10;
  public shoulder_auton(arm a) {
    // Use addRequirements() here to declare subsystem dependencies.
    arm =a;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTicks = arm.get_shoulder1_encoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = (Math.PI/3)- (arm.tick_to_rads(arm.get_shoulder1_encoder()-startTicks))/ (Math.PI/12);
    System.out.println(arm.get_shoulder1_encoder()-startTicks);
    double output = max * error;
    output = MathUtil.clamp(output, -0.10, 0.10);
    arm.move_shoulder(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.move_shoulder(-Constants.G_conpensate);
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
