// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  private final WPI_TalonFX frontleft = new WPI_TalonFX(Constants.frontleft);
  private final WPI_TalonFX backleft = new WPI_TalonFX(Constants.backleft);
  private final WPI_TalonFX frontright = new WPI_TalonFX(Constants.frontright);
  private final WPI_TalonFX backright = new WPI_TalonFX(Constants.backright);

  /*private final Encoder FL_Encoder = new Encoder(null, null);
  private final Encoder FR_Encoder = new Encoder(null, null);
  private final Encoder BL_Encoder = new Encoder(null, null);
  private final Encoder BR_Encoder = new Encoder(null, null);
  */
  
  
 // navX MXP using SPI
AHRS m_gyro = new AHRS(Port.kMXP);

  // Locations of the wheels relative to the robot center.
Translation2d m_frontLeftLocation = new Translation2d(0.4572, 0.4572);
Translation2d m_frontRightLocation = new Translation2d(0.4572, -0.4572);
Translation2d m_backLeftLocation = new Translation2d(-0.4572, 0.4572);
Translation2d m_backRightLocation = new Translation2d(-0.4572, -0.4572);

// Creating my kinematics object using the wheel locations.
MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(
  m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
);

/*_______________________________________________________________________________________
Example chassis speeds: 1 meter per second forward, 3 meters
// per second to the left, and rotation at 1.5 radians per second counterclockwise.
ChassisSpeeds speeds = new ChassisSpeeds(0.5, 1, Math.PI/2);

// Convert to wheel speeds
MecanumDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);

// Get the individual wheel speeds
double frontLeft = wheelSpeeds.frontLeftMetersPerSecond;
double frontRight = wheelSpeeds.frontRightMetersPerSecond;
double backLeft = wheelSpeeds.rearLeftMetersPerSecond;
double backRight = wheelSpeeds.rearRightMetersPerSecond;
_______________________________________________________________________________________*/


// The desired field relative speed here is 2 meters per second
// toward the opponent's alliance station wall, and 2 meters per
// second toward the left field boundary. The desired rotation
// is a quarter of a rotation per second counterclockwise. The current
// robot angle is 45 degrees.
ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
  2.0, 2.0, Math.PI/2.0, Rotation2d.fromDegrees(45.0));

// Now use this in our kinematics
MecanumDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);

  /** Creates a new DriveTrain. */
  public DriveTrain() {


    TalonSRXConfiguration config1 = new TalonSRXConfiguration();
      config1.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;
      
    initMotor(backleft);
    initMotor(frontleft);
    initMotor(backright);
    initMotor(frontright);
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
      // Get my wheel positions
  var wheelPositions = new MecanumDriveWheelPositions(
    frontleft.getSelectedSensorPosition(), frontright.getSelectedSensorPosition(),
    backleft.getSelectedSensorPosition(), backright.getSelectedSensorPosition());

  // Get the rotation of the robot from the gyro.
  var gyroAngle = m_gyro.getRotation2d();

  // Update the pose
  Pose2d m_pose = m_odometry.update(gyroAngle, wheelPositions);

  //Not sure if this works but we try
  m_odometry.resetPosition(gyroAngle, wheelPositions, m_pose);
  }

// Creating my odometry object from the kinematics object and the initial wheel positions.
// Here, our starting pose is 5 meters along the long end of the field and in the
// center of the field along the short end, facing the opposing alliance wall.
MecanumDriveOdometry m_odometry = new MecanumDriveOdometry(
  m_kinematics,
  m_gyro.getRotation2d(),
  new MecanumDriveWheelPositions(
    frontleft.getSelectedSensorPosition(), frontright.getSelectedSensorPosition(),
    backleft.getSelectedSensorPosition(), backright.getSelectedSensorPosition()
  ),
  new Pose2d(0, 0, new Rotation2d())
);

//re-add the configs once we are on SRX
public void initMotor(WPI_TalonFX motor){
  motor.setNeutralMode(NeutralMode.Brake);
}

public void MechDrive(double x, double y, double z, Rotation2d theta ){
  MecanumDrive m_drive = new MecanumDrive(frontleft, backleft, frontright, backleft);
  m_drive.driveCartesian(x, y, z, m_gyro.getRotation2d());
}

public Rotation2d geRotation2d(){
  return m_gyro.getRotation2d();
}

}
