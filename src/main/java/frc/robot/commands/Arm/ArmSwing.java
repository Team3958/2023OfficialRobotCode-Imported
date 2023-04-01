// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.math.MathUtil;
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
    if (xc.getLeftY()< 0){
      Arm.move_shoulder((add_dead_zone(xc.getLeftY(), 0.12))*0.3);
    }
    else if(xc.getLeftY()>0){
      Arm.move_shoulder(Math.pow(add_dead_zone(xc.getLeftY(), 0.12),2)*0.25);
    }

    Arm.move_extend(-xc.getRightY()*0.5);
  }

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Arm.rest_motor();
  }
  private double add_dead_zone(double input, double dead){
    double newRange = 1-dead;
    double direction = input/Math.abs(input);
    if (Math.abs(input) < dead){
      return 0;
    }
    input = (Math.abs(input)-dead) * direction/ newRange;
    input = MathUtil.clamp(input, -1, 1);
    return input;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
