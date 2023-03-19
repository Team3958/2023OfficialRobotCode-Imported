// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.Driving;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Extake;
import frc.robot.commands.Final_Auton_Drive;
import frc.robot.commands.PID_Tuning_Command;
import frc.robot.commands.Take;
import frc.robot.commands.last_try_auton;
import frc.robot.commands.pray_for_ramp;
import frc.robot.commands.Arm.ArmSwing;
import frc.robot.commands.Arm.rachet;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.arm;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.Arm.servo;
import frc.robot.subsystems.Drive_PID.PID_Drive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DriveTrain m_dt = new DriveTrain();
  private final XboxController m_driver = new XboxController(Constants.XboxPort);
  private final XboxController m_operator = new XboxController(Constants.XboxPort2);
  private final intake m_intake = new intake();
  private final Robot robot = new Robot();
  private final arm m_arm = new arm();
  private final servo m_Servo = new servo();


  private final PID_Drive d = new PID_Drive();

  private final PID_Tuning_Command tuning = new PID_Tuning_Command(m_dt);
  private final Extake m_extake = new Extake(m_intake);
  private final Take m_take = new Take(m_intake);
  private final Driving m_driving = new Driving(m_dt, m_driver);
  private final ArmSwing m_swinging = new ArmSwing(m_arm, m_operator);
  private final Final_Auton_Drive m_auton_drive = new Final_Auton_Drive(d, 0, 0.5, 2);
  private final rachet m_RachetComand = new rachet(m_Servo, m_operator);
  private final last_try_auton plz_God = new last_try_auton(m_dt);
  private final pray_for_ramp ramp = new pray_for_ramp(m_dt);


  //private final Command plz = auto; 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings(
    
    );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // setting defult to PID for testing
    m_dt.setDefaultCommand(m_driving);
    //m_dt.setDefaultCommand(tuning);
    m_arm.setDefaultCommand(m_swinging);
    m_Servo.setDefaultCommand(m_RachetComand);

    new JoystickButton(m_operator, Constants.XboxPortB)
      .toggleOnTrue(m_take);

    new JoystickButton(m_operator, Constants.XboxPortX)
      .toggleOnTrue(m_extake);
    

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return ramp;
  }
}
