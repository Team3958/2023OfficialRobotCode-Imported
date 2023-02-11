package frc.robot.commands.Auton;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;

/**
 * This holonomic drive controller can be used to follow trajectories using a holonomic drive train
 * (i.e. swerve or mecanum). Holonomic trajectory following is a much simpler problem to solve
 * compared to skid-steer style drivetrains because it is possible to individually control forward,
 * sideways, and angular velocity.
 *
 * <p>The holonomic drive controller takes in one PID controller for each direction, forward and
 * sideways, and one profiled PID controller for the angular direction. Because the heading dynamics
 * are decoupled from translations, users can specify a custom heading that the drivetrain should
 * point toward. This heading reference is profiled for smoothness.
 */
@SuppressWarnings("MemberName")
public class HolonomicController {
  private Pose2d m_poseError = new Pose2d();
  private Rotation2d m_rotationError = new Rotation2d();
  private Pose2d m_poseTolerance = new Pose2d();
  private boolean m_enabled = true;

  private final PIDController m_xController;
  private final PIDController m_yController;
  private final ProfiledPIDController m_thetaController;

  /**
   * Constructs a holonomic drive controller.
   *
   * @param xController A PID Controller to respond to error in the field-relative x direction.
   * @param yController A PID Controller to respond to error in the field-relative y direction.
   * @param thetaController A profiled PID controller to respond to error in angle.
   */
  @SuppressWarnings("ParameterName")
  public HolonomicController(
      PIDController xController, PIDController yController, ProfiledPIDController thetaController) {
    m_xController = xController;
    m_yController = yController;
    m_thetaController = thetaController;

    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns true if the pose error is within tolerance of the reference.
   *
   * @return True if the pose error is within tolerance of the reference.
   */
  public boolean atReference() {
    final var eTranslate = m_poseError.getTranslation();
    final var eRotate = m_rotationError;
    final var tolTranslate = m_poseTolerance.getTranslation();
    final var tolRotate = m_poseTolerance.getRotation();
    return Math.abs(eTranslate.getX()) < tolTranslate.getX()
        && Math.abs(eTranslate.getY()) < tolTranslate.getY()
        && Math.abs(eRotate.getRadians()) < tolRotate.getRadians();
  }

  /**
   * Sets the pose error which is considered tolerance for use with atReference().
   *
   * @param tolerance The pose error which is tolerable.
   */
  public void setTolerance(Pose2d tolerance) {
    m_poseTolerance = tolerance;
  }

  double feedbackTotal;
  int count = 1;

  /**
   * Returns the next output of the holonomic drive controller.
   *
   * @param currentPose The current pose.
   * @param poseRef The desired pose.
   * @param linearVelocityRefMeters The linear velocity reference.
   * @param angleRef The angular reference.
   * @param absoluteHeading The absolute heading of the robot.
   * @return The next output of the holonomic drive controller.
   */
  @SuppressWarnings("LocalVariableName")
  public ChassisSpeeds calculate(
      Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters, Rotation2d angleRef, Rotation2d absoluteHeading, double angularVelocityRef) {
    // Calculate feedforward velocities (field-relative).
    double xFF = linearVelocityRefMeters * Math.cos(absoluteHeading.getRadians());
    double yFF = -1 * linearVelocityRefMeters * Math.sin(absoluteHeading.getRadians());
    // System.out.println("Xv Feed Forward: " + xFF);
    // System.out.println("Yv Feed Forward: " + yFF);

    double thetaFF =  m_thetaController.calculate(currentPose.getRotation().getRadians(), angleRef.getRadians());

    //System.out.println("ThetaFF (" + angleRef.getDegrees() +  "): " + thetaFF + " rad/s");
    /** 
    double thetaFF =
        angularVelocityRef;
    */

    m_poseError = poseRef.relativeTo(currentPose);
    m_rotationError = angleRef.minus(currentPose.getRotation());

    if (!m_enabled) {
      return ChassisSpeeds.fromFieldRelativeSpeeds(xFF, yFF, thetaFF, currentPose.getRotation());
    }

    // Calculate feedback velocities (based on position error).
    double xFeedback = m_xController.calculate(currentPose.getX(), poseRef.getX());
    double yFeedback = m_yController.calculate(currentPose.getY(), poseRef.getY());
    //System.out.println("GoalRot: " + angleRef.getDegrees() + " degrees");

    count++;
    // Return next output.
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        xFF + xFeedback, yFF + yFeedback, thetaFF, currentPose.getRotation());
  }

  /**
   * Returns the next output of the holonomic drive controller.
   *
   * @param currentPose The current pose.
   * @param desiredState The desired trajectory state.
   * @param angleRef The desired end-angle.
   * @return The next output of the holonomic drive controller.
   */
  public ChassisSpeeds calculate(
      Pose2d currentPose, Trajectory.State desiredState, Rotation2d angleRef, Trajectory.State absoluteHeading) {
    return calculate(
        currentPose, desiredState.poseMeters, desiredState.velocityMetersPerSecond, angleRef, absoluteHeading.poseMeters.getRotation(), absoluteHeading.velocityMetersPerSecond);
  }

  /**
   * Enables and disables the controller for troubleshooting problems. When calculate() is called on
   * a disabled controller, only feedforward values are returned.
   *
   * @param enabled If the controller is enabled or not.
   */
  public void setEnabled(boolean enabled) {
    m_enabled = enabled;
  }

  public double angleTo360(double angle){
    double output;
    if (angle < 0){
      output = 360 - Math.abs(angle);
    } else {
      output = angle;
    }

    return output;
  }
  
}