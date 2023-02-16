// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class Driving extends CommandBase {
  private final DriveTrain dt;
  private final XboxController xc;


  /** Creates a new Driving. */
  public Driving(DriveTrain d, XboxController x) {
    dt = d;
    xc = x;

    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(d);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dt.zeroYaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    dt.MechDrive(-xc.getRightX(), xc.getLeftX(), xc.getLeftY());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.MechDrive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
