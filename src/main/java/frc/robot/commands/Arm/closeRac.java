// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.i2c;
import frc.robot.subsystems.Arm.servo;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class closeRac extends InstantCommand {
  private servo sv;
  private i2c port;
  public closeRac(servo s, i2c i) {
    // Use addRequirements() here to declare subsystem dependencies.
    sv = s;
    port = i;
    addRequirements(port);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sv.close();
    port.close();
  }
}
