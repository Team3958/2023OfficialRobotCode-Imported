// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AutonStuff;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutonKinematics extends SubsystemBase {

  private SimpleMatrix m_inverseKinematics;
  private final SimpleMatrix m_forwardKinematics;

  private final Translation2d m_frontLeftWheelMeters;
  private final Translation2d m_frontRightWheelMeters;
  private final Translation2d m_rearLeftWheelMeters;
  private final Translation2d m_rearRightWheelMeters;

  private Translation2d m_prevCoR = new Translation2d();
  
  /** Creates a new AutonKinematics. */
  public AutonKinematics
  (Translation2d frontLeftWheelMeters, 
  Translation2d frontRightWheelMeters, 
  Translation2d rearLeftWheelMeters,
  Translation2d rearRightWheelMeters ) {  
 
  {
  m_frontLeftWheelMeters = frontLeftWheelMeters;
  m_frontRightWheelMeters = frontRightWheelMeters;
  m_rearLeftWheelMeters = rearLeftWheelMeters;
  m_rearRightWheelMeters = rearRightWheelMeters;

  m_inverseKinematics = new SimpleMatrix(4, 3);

  setInverseKinematics(
      frontLeftWheelMeters, frontRightWheelMeters, rearLeftWheelMeters, rearRightWheelMeters);
  m_forwardKinematics = m_inverseKinematics.pseudoInverse();

  MathSharedStore.reportUsage(MathUsageId.kKinematics_MecanumDrive, 1);
  }
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public MecanumDriveWheelSpeeds toWheelSpeeds(
      ChassisSpeeds chassisSpeeds, Translation2d centerOfRotationMeters) {
    // We have a new center of rotation. We need to compute the matrix again.
    if (!centerOfRotationMeters.equals(m_prevCoR)) {
      var fl = m_frontLeftWheelMeters.minus(centerOfRotationMeters);
      var fr = m_frontRightWheelMeters.minus(centerOfRotationMeters);
      var rl = m_rearLeftWheelMeters.minus(centerOfRotationMeters);
      var rr = m_rearRightWheelMeters.minus(centerOfRotationMeters);

      setInverseKinematics(fl, fr, rl, rr);
      m_prevCoR = centerOfRotationMeters;
    }

    var chassisSpeedsVector = new SimpleMatrix(3, 1);
    chassisSpeedsVector.setColumn(
        0,
        0,
        chassisSpeeds.vxMetersPerSecond,
        chassisSpeeds.vyMetersPerSecond,
        chassisSpeeds.omegaRadiansPerSecond);

    var wheelsMatrix = m_inverseKinematics.mult(chassisSpeedsVector);
    return new MecanumDriveWheelSpeeds(
        wheelsMatrix.get(0, 0),
        wheelsMatrix.get(1, 0),
        wheelsMatrix.get(2, 0),
        wheelsMatrix.get(3, 0));
  }

  public MecanumDriveWheelSpeeds toWheelSpeeds(ChassisSpeeds chassisSpeeds) {
    return toWheelSpeeds(chassisSpeeds, new Translation2d());
  }

  public ChassisSpeeds toChassisSpeeds(MecanumDriveWheelSpeeds wheelSpeeds) {
    var wheelSpeedsMatrix = new SimpleMatrix(4, 1);
    wheelSpeedsMatrix.setColumn(
        0,
        0,
        wheelSpeeds.frontLeftMetersPerSecond,
        wheelSpeeds.frontRightMetersPerSecond,
        wheelSpeeds.rearLeftMetersPerSecond,
        wheelSpeeds.rearRightMetersPerSecond);
    var chassisSpeedsVector = m_forwardKinematics.mult(wheelSpeedsMatrix);

    return new ChassisSpeeds(
        chassisSpeedsVector.get(0, 0),
        chassisSpeedsVector.get(1, 0),
        chassisSpeedsVector.get(2, 0));
  }


  private void setInverseKinematics(
      Translation2d fl, Translation2d fr, Translation2d rl, Translation2d rr) {
        m_inverseKinematics.setRow(0, 0, 1, -1, -(fl.getX() + fl.getY()));
        m_inverseKinematics.setRow(1, 0, 1, 1, fr.getX() - fr.getY());
        m_inverseKinematics.setRow(2, 0, 1, 1, rl.getX() - rl.getY());
        m_inverseKinematics.setRow(3, 0, 1, -1, -(rr.getX() + rr.getY()));
  }

}
