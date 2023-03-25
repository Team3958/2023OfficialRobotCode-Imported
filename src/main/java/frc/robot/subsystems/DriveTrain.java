// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXPIDSetConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenixpro.configs.Slot0Configs;
import com.ctre.phoenixpro.configs.VoltageConfigs;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  private final WPI_TalonFX frontleft = new WPI_TalonFX(Constants.frontleft);
  private final WPI_TalonFX backleft = new WPI_TalonFX(Constants.backleft);
  private final WPI_TalonFX frontright = new WPI_TalonFX(Constants.frontright);
  private final WPI_TalonFX backright = new WPI_TalonFX(Constants.backright);
  private final XboxController xc = new XboxController(Constants.XboxPort);
  private int dpad_angle;

  private ProfiledPIDController fl;
  private double some;
  private double p;
  private double i;
  private double d;


 // navX MXP using SPI
public AHRS m_gyro = new AHRS(Port.kMXP);

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
  3.0, 1.0, Math.PI, Rotation2d.fromDegrees(45.0));

// Now use this in our kinematics
MecanumDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);
MecanumDrive m_drive;

  /** Creates a new DriveTrain. */
  public DriveTrain() {

    TalonFXConfiguration config1 = new TalonFXConfiguration();
      config1.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;

      //config1.peakCurrentLimit = Constants.currentlimit_DriveTrain;

      config1.slot0.kF = Constants.kF;
      config1.slot0.kP = Constants.kP;
      config1.slot0.kI = Constants.kI;
      config1.slot0.kD = Constants.kD;
    
      
    initMotor(frontleft, config1);
    initMotor(backleft, config1);
    initMotor(frontright,config1);
    initMotor(backright, config1);
    
    //m_drive =  new MecanumDrive(frontleft, backleft, backright, frontright);
  
    backleft.setInverted(true);
    frontleft.setInverted(true);
    
    fl = new ProfiledPIDController(Constants.fl_kP, Constants.fl_kI, Constants.fl_kD, new TrapezoidProfile.Constraints(4, 2));
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

  SmartDashboard.putNumber("yaw", getYaw());
  SmartDashboard.putNumber("pitch", getPitch());
  SmartDashboard.putNumber("roll", getRoll());
  SmartDashboard.putNumber("Heading", getAngle());

  SmartDashboard.putNumber("frontLeft speed", frontleft.getMotorOutputPercent());
  SmartDashboard.putNumber("frontRight speed", frontright.getMotorOutputPercent());
  SmartDashboard.putNumber("backLeft speed", backleft.getMotorOutputPercent());
  SmartDashboard.putNumber("backRight speed", backright.getMotorOutputPercent());

  SmartDashboard.putNumber("x", getCurrentX());
  SmartDashboard.putNumber("y", getCurrentY());

  SmartDashboard.putNumber("fl encoder value", get_fl_encoder());
  SmartDashboard.putNumber("fl distance travelled", tick_to_distance(get_fl_encoder()));
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
public void initMotor(WPI_TalonFX motor, TalonFXConfiguration config){
  //motor.configFactoryDefault();
  motor.setNeutralMode(NeutralMode.Brake);
  motor.configAllSettings(config);
  motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,0);
  motor.enableVoltageCompensation(true);
  motor.configVoltageCompSaturation(8);
}
public TalonFX get_fl_motor(){
  return frontleft;
}

/*public void MechDrive(double x, double y, double z){
  //x = Math.pow(x, 3) ;
  //y = Math.pow(y, 3) ;
  //z = Math.pow(z, 3) ;
  m_drive.driveCartesian(x, y, z);
}*/
  public double get_fl_encoder(){
    return frontleft.getSelectedSensorPosition(); 
  }
  public double get_fr_encoder(){
    return frontright.getSelectedSensorPosition(); 
  }
  public double get_bl_encoder(){
    return backleft.getSelectedSensorPosition(); 
  }
  public double get_br_encoder(){
    return backright.getSelectedSensorPosition(); 
  }

  public double tick_to_distance(double ticks){
    double motor_rotation = ticks /Constants.kEncoderTicksPerRev;
    double wheel_rotaion = motor_rotation* Constants.kGearRatio;
    double distance = 2* Math.PI* Constants.kWheelRadiusMeters * wheel_rotaion;
    return distance;
  } 

  public void telop_drive(double x, double y, double z){
    double fl_drive = x + y+ z;
    double fr_drive = x - y- z;
    double bl_drive = x - y+ z;
    double br_drive = x + y-z;
    double sum = Math.abs(br_drive) + Math.abs(fl_drive) + Math.abs(bl_drive) + Math.abs(fr_drive);
  
    /*fl_drive/= 3;
    fr_drive/=3;
    bl_drive/=3;
    br_drive/=3;*/

    MathUtil.clamp(fl_drive, -1, 1);
    MathUtil.clamp(fr_drive, -1, 1);
    MathUtil.clamp(bl_drive, -1, 1);
    MathUtil.clamp(br_drive, -1, 1);


    if (sum == 0){
      frontleft.set(ControlMode.PercentOutput, 0);
      frontright.set(ControlMode.PercentOutput, 0);
      backleft.set(ControlMode.PercentOutput, 0);
      backright.set(ControlMode.PercentOutput, 0);
    }

    frontleft.set(ControlMode.PercentOutput, fl_drive);
    frontright.set(ControlMode.PercentOutput, fr_drive);
    backleft.set(ControlMode.PercentOutput, bl_drive);
    backright.set(ControlMode.PercentOutput, fl_drive);
}

public void VelocityMode(int speed){
    frontleft.set(ControlMode.Velocity, speed);
    frontright.set(ControlMode.Velocity, speed);
    backleft.set(ControlMode.Velocity, speed);
    backright.set(ControlMode.Velocity, speed);
}

public Rotation2d getRotation2d(){
  return m_gyro.getRotation2d();
}
/*public void drive_by_voltage(double volts){
  frontleft.setVoltage(volts);
  frontright.setVoltage(volts);
  backleft.setVoltage(volts);
  backright.setVoltage(volts);
}*/
public void drive_by_pose(double measurement){
  frontleft.set(ControlMode.Position, measurement);
}

public void drive_by_percent(double output1,double output2, double output3, double output4){
  frontleft.set(ControlMode.PercentOutput, output1);
  frontright.set(ControlMode.PercentOutput, output2);
  backleft.set(ControlMode.PercentOutput, output3);
  backright.set(ControlMode.PercentOutput, output4);
}

public void zeroYaw(){
  m_gyro.zeroYaw();
}

public void zero(){
  m_gyro.reset();
}

public double getAngle(){
  return m_gyro.getAngle();
}

public float getYaw(){
  return m_gyro.getYaw();
}

public double getPitch(){
  return m_gyro.getPitch();
}

public double getRoll(){
  return m_gyro.getRoll();
}

public double getCurrentX(){
  return m_odometry.getPoseMeters().getX();
}

public double getCurrentY(){
  return m_odometry.getPoseMeters().getY();
}


public void tankish_drive(double L, double R){
  frontleft.set(ControlMode.PercentOutput, L);
  backleft.set(ControlMode.PercentOutput, L);
  frontright.set(ControlMode.PercentOutput, R);
  backright.set(ControlMode.PercentOutput, R);
}

}
