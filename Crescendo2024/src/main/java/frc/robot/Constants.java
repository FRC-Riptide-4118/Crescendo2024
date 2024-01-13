// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double DEG_PER_RAD = 180 / Math.PI;

  public static class DriveConstants {

    public static enum MotorPosition {
      C_FRONT_LEFT_DRIVE, C_FRONT_RIGHT_DRIVE, C_REAR_LEFT_DRIVE, C_REAR_RIGHT_DRIVE,
      C_FRONT_LEFT_STEER, C_FRONT_RIGHT_STEER, C_REAR_LEFT_STEER, C_REAR_RIGHT_STEER

    };

    public static final int front_left_drive_id  = 1;
    public static final int front_right_drive_id = 2;
    public static final int rear_left_drive_id   = 3;
    public static final int rear_right_drive_id  = 4;

    public static final int front_left_steer_id  = 5;
    public static final int front_right_steer_id = 6;
    public static final int rear_left_steer_id   = 7;
    public static final int rear_right_steer_id  = 8;

    public static final double TICKS_PER_ROTATION = 21.64;
    public static final double DEGREES_PER_TICK = 360 / TICKS_PER_ROTATION;
    
    public static final double steer_kP = 1;
    public static final double steer_kI = 0;
    public static final double steer_kD = 0;
    public static final double steer_kF = 0;

  }

  public static class ControllerConstants {

    public static final int driver_controller_id = 0;

  }

}
