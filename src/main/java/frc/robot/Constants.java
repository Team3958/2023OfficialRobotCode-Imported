// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

//Xbox Constants
    public static final int XboxPort = 1;
    public static final int Y = 4;

//Drivetrain Constants
    public static final int backleft = 03;
    public static final int frontright = 20;
    public static final int backright = 02;
    public static final int frontleft = 01;
    public static final double kF = 0;
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

//Arm Constants
    public static final int shoulder1 = 5;
    public static final int shoulder2 = 6;
    public static final int extend = 0;
    public static final int wrist = 0;
    public static final int grip = 0;

    public static final double kWheelRadiusInches = 3;
        public static final double kWheelRadiusMeters = 0.0762;
        public static final int kEncoderTicksPerRev = 2048;

        // Wheel positions in FEET
        public static final double kFrontRight_y = -1;
        public static final double kFrontRight_x = 0.875;
        public static final double kFrontLeft_y = 1;
        public static final double kFrontLeft_x = 0.875;
        public static final double kBackLeft_y = 1;
        public static final double kBackLeft_x = -0.875;
        public static final double kBackRight_y = -1;
        public static final double kBackRight_x = -0.875;

        // Wheel Gearing
        public static final double kGearRatio = 1/10;

        // Speed Constraints
        public static final double kMaxSpeedFeetPerSecond = 5.0;
        public static final double kMaxAccelerationFeetPerSecond = 10.0;

        // PID Constants
        public static final double fl_kP = 0.2;
        public static final double fl_kI = 0;
        public static final double fl_kD = 0.05;

        public static final double bl_kP = 0.2;// 0.127
        public static final double bl_kI = 0;
        public static final double bl_kD = 0.05;

        public static final double fr_kP = 0.2;
        public static final double fr_kI = 0;
        public static final double fr_kD = 0.05;

        public static final double br_kP = 0.2;
        public static final double br_kI = 0;
        public static final double br_kD = 0.05;
        
        // Characterization Data
        public static final double kS = 0.495;// 0.495
        public static final double kV = 2.04;// 2.04
        public static final double kA = 0.0755;// 0.0755

        
        public static final int intake1 = 0;
        public static final int intake2 = 0;
        public static final int XboxPortA = 0;
        public static final int XboxPortB = 0;
        public static final int XboxPort2 = 2;



        public static final double turn_kP = 0.2;
        public static final double turn_kI = 0;
        public static final double turn_kD = 0;

        public static final double ShoulderP = 0.2;
        public static final double ShoulderI = 0;
        public static final double ShoulderD = 0;

        public static final double ExtendP = 0.2;
        public static final double ExtendI = 0;
        public static final double ExtendD = 0;

        public static final double WristP = 0.2;
        public static final double WristI = 0;
        public static final double WristD = 0;

        public static final int startingLength = 0;
        public static final int RevPerMeter = 0;    

}
