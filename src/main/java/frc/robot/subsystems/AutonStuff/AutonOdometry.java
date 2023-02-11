// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AutonStuff;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutonOdometry extends SubsystemBase {

  private final AutonKinematics m_kinematics;
  private Pose2d m_poseMeters;
  private double m_prevTimeSeconds = -1;

  private Rotation2d m_gyroOffset;
  private Rotation2d m_previousAngle;

  /** Creates a new AutonOdometry. */
  public AutonOdometry(AutonKinematics kinematics, Rotation2d gyroAngle, Pose2d initialPoseMeters) {

    m_kinematics = kinematics;
    m_poseMeters = initialPoseMeters;
    m_gyroOffset = m_poseMeters.getRotation().minus(gyroAngle);
    m_previousAngle = initialPoseMeters.getRotation();
    MathSharedStore.reportUsage(MathUsageId.kOdometry_MecanumDrive, 1);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public AutonOdometry(AutonKinematics kinematics, Rotation2d gyroAngle) {
    this(kinematics, gyroAngle, new Pose2d());
  }

  public void resetPosition(Pose2d poseMeters, Rotation2d gyroAngle) {
    m_poseMeters = poseMeters;
    m_previousAngle = poseMeters.getRotation();
    m_gyroOffset = m_poseMeters.getRotation().minus(gyroAngle);
  }

  public Pose2d getPoseMeters() {
    return m_poseMeters;
  }

  public Pose2d updateWithTime(
      double currentTimeSeconds, Rotation2d gyroAngle, MecanumDriveWheelSpeeds wheelSpeeds) {
    double period = m_prevTimeSeconds >= 0 ? currentTimeSeconds - m_prevTimeSeconds : 0.0;
    m_prevTimeSeconds = currentTimeSeconds;

    var angle = gyroAngle.plus(m_gyroOffset);

    var chassisState = m_kinematics.toChassisSpeeds(wheelSpeeds);
    var newPose =
        m_poseMeters.exp(
            new Twist2d(
                chassisState.vxMetersPerSecond * period,
                chassisState.vyMetersPerSecond * period,
                angle.minus(m_previousAngle).getRadians()));

    m_previousAngle = angle;
    m_poseMeters = new Pose2d(newPose.getTranslation(), angle);
    return m_poseMeters;
  }

  public Pose2d update(Rotation2d gyroAngle, MecanumDriveWheelSpeeds wheelSpeeds) {
    return updateWithTime(WPIUtilJNI.now() * 1.0e-6, gyroAngle, wheelSpeeds);
  }

}
