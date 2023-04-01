// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Extake;
import frc.robot.commands.Final_Auton_Drive;
import frc.robot.commands.PID_Tuning_Command;
import frc.robot.commands.Take;
import frc.robot.commands.last_try_auton;
import frc.robot.commands.pray_for_ramp;
import frc.robot.commands.Arm.ArmSwing;
import frc.robot.commands.Arm.auton_extake;
import frc.robot.commands.Arm.closeRac;
import frc.robot.commands.Arm.extension_auton;
import frc.robot.commands.Arm.low_cube;
import frc.robot.commands.Arm.openRac;
import frc.robot.commands.Arm.rachetOpen;
import frc.robot.commands.Arm.shoulder_auton;
import frc.robot.commands.AutonRoutines.just_taxi;
import frc.robot.commands.Driving.DriveToDistance;
import frc.robot.commands.Driving.Driving;
import frc.robot.commands.Driving.StrafeToDistance;
import frc.robot.commands.Driving.TurnToAngle;
import frc.robot.commands.Driving.drive_by_encoder;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.arm;
import frc.robot.subsystems.i2c;
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
  private final i2c m_port = new i2c();


  private final PID_Drive d = new PID_Drive();

  private final PID_Tuning_Command tuning = new PID_Tuning_Command(m_dt);
  private final Extake m_extake = new Extake(m_intake);
  private final Take m_take = new Take(m_intake);
  private final Driving m_driving = new Driving(m_dt, m_driver);
  private final ArmSwing m_swinging = new ArmSwing(m_arm, m_operator);
  private final Final_Auton_Drive m_auton_drive = new Final_Auton_Drive(d, 0, 0.5, 2);
  private final openRac m_openRachet = new openRac(m_Servo, m_port);
  private final closeRac m_closeRachet = new closeRac(m_Servo, m_port);
  private final last_try_auton plz_God = new last_try_auton(m_dt);
  private final pray_for_ramp ramp = new pray_for_ramp(m_dt);
  private final DriveToDistance m_dtd = new DriveToDistance(m_dt, 3.0);
  private final StrafeToDistance m_std = new StrafeToDistance(m_dt, ()-> 3.0);
  private final TurnToAngle m_tta = new TurnToAngle(m_dt, ()-> 90);
  private final drive_by_encoder m_d = new drive_by_encoder(m_dt, 2.0);
  private final extension_auton m_extend = new extension_auton(m_arm, 10);
  private final shoulder_auton mShoulder_auton = new shoulder_auton(m_arm);
  private final just_taxi m_taxi = new just_taxi(m_dt);
  private final low_cube m_lowCube = new low_cube(m_arm, m_intake, m_dt);
  private final auton_extake m_ex = new auton_extake(m_intake);


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
    //m_Servo.setDefaultCommand(m_RachetComand);
    

    new JoystickButton(m_operator, Constants.XboxPortLB)
      .whileTrue(m_openRachet);
    new JoystickButton(m_operator, Constants.XboxPortRB)
      .whileTrue(m_closeRachet);

    new JoystickButton(m_operator, Constants.XboxPortB)
      .whileTrue(m_take);

    new JoystickButton(m_operator, Constants.XboxPortX)
      .whileTrue(m_extake);

    new JoystickButton(m_driver, Constants.XboxPortA)
      .toggleOnTrue(m_dtd);
    
    new JoystickButton(m_driver, Constants.XboxPortB)
      .toggleOnTrue(m_std);

    new JoystickButton(m_driver, Constants.XboxPortY)
      .toggleOnTrue(m_tta);
    new JoystickButton(m_operator, Constants.XboxPortY)
    .whileTrue(mShoulder_auton);
    new JoystickButton(m_operator, Constants.XboxPortA)
      .whileTrue(m_extend);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_lowCube;
  }
}
