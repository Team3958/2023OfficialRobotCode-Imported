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

    public static final int backleft = 1;
    public static final int frontright = 2;
    public static final int backright = 3;
    public static final int frontleft = 0;
    public static final int XboxPort = 0;

    /*
     * The idea with this is that we line up facing one of the APRIL Tags at the start of each game
     * Set to 1 for Tags 1 and 6
     * Set to 2 for Tags 2 and 7
     * Set to 3 for Tags 3 and 8
     */
    public static final int AutonomousRoutine = 1; 
    
}
