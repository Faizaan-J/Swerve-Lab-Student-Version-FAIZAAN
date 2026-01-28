// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class SwerveConstants {
    public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(21.5);
    public static final double WHEEL_BASE_METERS = Units.inchesToMeters(21.5);
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4.0);
    
    public static final Translation2d FRONT_LEFT_POSITION = 
        new Translation2d(WHEEL_BASE_METERS / 2.0, TRACK_WIDTH_METERS / 2.0);
    public static final Translation2d FRONT_RIGHT_POSITION = 
        new Translation2d(WHEEL_BASE_METERS / 2.0, -TRACK_WIDTH_METERS / 2.0);
    public static final Translation2d BACK_LEFT_POSITION = 
        new Translation2d(-WHEEL_BASE_METERS / 2.0, TRACK_WIDTH_METERS / 2.0);
    public static final Translation2d BACK_RIGHT_POSITION = 
        new Translation2d(-WHEEL_BASE_METERS / 2.0, -TRACK_WIDTH_METERS / 2.0);
    
    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
        FRONT_LEFT_POSITION,
        FRONT_RIGHT_POSITION,
        BACK_LEFT_POSITION,
        BACK_RIGHT_POSITION
    );
    
    public static final double DRIVE_GEAR_RATIO = 6.75;
    public static final double TURN_GEAR_RATIO = 150.0 / 7.0;
    
    public static final double MAX_SPEED_MPS = 4.5;
    public static final double MAX_ANGULAR_SPEED = Math.PI * 2;

    /***
     * Worry about this later. PID tuning is more useful when working with a real drivetrain. Not too useful for simulations.
     */
    
    public static final double DRIVE_KP = 1.0;  // TODO: TUNE THIS!
    public static final double DRIVE_KI = 0.0;  // TODO: TUNE THIS!
    public static final double DRIVE_KD = 0.1;  // TODO: TUNE THIS!
    
    public static final double TURN_KP = 1.0;   // TODO: TUNE THIS!
    public static final double TURN_KI = 0.0;   // TODO: TUNE THIS!
    public static final double TURN_KD = 0.1;   // TODO: TUNE THIS!
  }
  
  public static final class IOConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    
    public static final int LEFT_X_AXIS = 0;
    public static final int LEFT_Y_AXIS = 1;
    public static final int RIGHT_X_AXIS = 4;
    
    public static final double DEADBAND = 0.1;
    
    public static final double SPEED_MULTIPLIER = 0.8;
  }

}