// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public class Arm_pid_sub extends ProfiledPIDSubsystem {
  /** Creates a new Arm_pid_sub. */
  public Arm_pid_sub() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            0.2,
            0,
            0.05,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(3, 1.5)));
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }
}
