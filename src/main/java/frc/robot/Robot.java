/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Auton.MPController;
import frc.robot.commands.Auton.Trajectories;

public class Robot extends TimedRobot {

  // Creates our Motion Profile Controller and Trajectories class
  Trajectories trajectories = new Trajectories();
  MPController mpController;

  public static Timer m_autoTimer = new Timer();

  public static Trajectory[] selectedTrajectory = new Trajectory[2];

  private RobotContainer m_RobotContainer;
  String selectedPath;

  double trajectoryTime;

  /**
   * Trajectory Paths. 
   */
  String rightFivePath = "paths/RightFiveFeet.csv";
  String forwardTenSpinPath = "paths/ForwardTenSpin.csv";
  String barrelRacePath = "paths/BarrelRacePath.csv";
  String forwardTenPath = "paths/ForwardTen.csv";
  String slalomPath = "paths/SlalomPath.csv";
  String circlePath = "paths/TestCircle.csv";
  String plz = "pathplanner/generatedCSV/New Path.csv";
  String safe = "pathplanner/generatedCSV/Test Path (safe).csv";
  String straight = "pathplanner/generatedCSV/Straight.csv";
  Command autoCommand;

  @Override
  public void robotInit() {
    
    m_RobotContainer = new RobotContainer();
    PortForwarder.add(5800, "10.39.58.11", 5800);
  }

  @Override
  public void robotPeriodic() {
   
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    
    Command m_autonomousCommand = m_RobotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    // Continue updating odometry while we use it in autonomous
    mpController.drive.periodic(); 

    if (autoCommand.isFinished()){
      m_autoTimer.stop();
    }
  }

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void disabledInit() {
    // Cancel any commands that were running
    
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }
}