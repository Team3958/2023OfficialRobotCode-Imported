// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Driving.drive_by_encoder;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.arm;
import frc.robot.subsystems.intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class low_cube extends SequentialCommandGroup {
  /** Creates a new low_cube. */
  private arm arm;
  private intake intake;
  private DriveTrain dt;
  public low_cube(arm a, intake i, DriveTrain d) {
    arm = a;
    intake = i;
    dt = d;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new shoulder_auton(arm),
      new auton_extake(intake),
      new drive_by_encoder(dt, -2.5)
    );
  }
}
