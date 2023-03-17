// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.arm;

public class ArmMove extends CommandBase {
  private arm Arm;
  private XboxController xc;
  /** Creates a new ArmSwing. */
  public ArmMove(arm a, XboxController x) {
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
    if (xc.getLeftX()< 0){
      Arm.move_shoulder((xc.getLeftX())*0.1);
    }
    else if(xc.getLeftX()>0){
      Arm.move_shoulder(Math.pow(xc.getLeftX(),2)*0.25);
    }

    if (xc.getBButtonPressed()){
      Arm.move_wrist(0.1);
    }
    else if(xc.getYButtonPressed()){
      Arm.move_wrist(-0.1);
    }
    else{
      Arm.move_wrist(0);
    }

    Arm.move_extend(xc.getRightY()*0.2);
  }

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Arm.rest_motor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
