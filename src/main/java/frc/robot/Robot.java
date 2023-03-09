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
    mpController = new MPController();

    // Sets the path to be driven. 
    selectedPath = straight;

    for (int i = 0; i < 2; i++){
      selectedTrajectory[i] = trajectories.getTrajectoryFromCSV(selectedPath)[i];
    }

    mpController.drive.setupMotorConfigs();

    trajectoryTime = selectedTrajectory[0].getTotalTimeSeconds();
    m_RobotContainer = new RobotContainer();
    System.out.println("Total Trajectory Time: " + trajectoryTime + "s");
    
  }

  @Override
  public void robotPeriodic() {
    mpController.drive.putEncoder();
    mpController.drive.putGyro();
    SmartDashboard.putNumber("X Pose (Ft): ", Units.metersToFeet(mpController.drive.getPose().getX()));
    SmartDashboard.putNumber("Y Pose (Ft): ", Units.metersToFeet(mpController.drive.getPose().getY()));
    SmartDashboard.putNumber("Rotation Pose (Degrees): ", mpController.drive.getPose().getRotation().getDegrees());
    mpController.drive.putWheelVelocities();
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    
    // Reset encoders
    mpController.drive.resetEncoders();

    // Initialize our odometry
    mpController.drive.initializeOdometry();

    // Ensure our odometry is at 0
    mpController.drive.reset();

    // Reset odometry to starting point of path
    mpController.drive.resetOdometry(selectedTrajectory[0].getInitialPose());

    // Update our odometry
    mpController.drive.periodic();    

    autoCommand = mpController.createTrajectoryFollowerCommand(selectedTrajectory[0], selectedTrajectory[1], 2.5);

    autoCommand.schedule();

    m_autoTimer.reset();
    m_autoTimer.start();
  
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
    mpController.drive.setOutputVelocity(new MecanumDriveWheelSpeeds());
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