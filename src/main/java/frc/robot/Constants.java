// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// FIRST imports
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Translation2d;

// REV imports
import com.revrobotics.CANSparkBase.IdleMode;

// Java imports
import java.util.*;

// Pheonix imports
import com.ctre.phoenix6.hardware.CANcoder;

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

    // Motors
    public static final int front_left_drive_id     = 1;
    public static final int front_right_drive_id    = 2;
    public static final int rear_left_drive_id      = 3;
    public static final int rear_right_drive_id     = 4;

    public static final int front_left_steer_id     = 5;
    public static final int front_right_steer_id    = 6;
    public static final int rear_left_steer_id      = 7;
    public static final int rear_right_steer_id     = 8;

    public static final int front_left_CANcoder_id  = 5;
    public static final int front_right_CANcoder_id = 6;
    public static final int rear_left_CANcoder_id   = 7;
    public static final int rear_right_CANcoder_id  = 8;

    public static final int pigeon_id = 9;

    public static final double TICKS_PER_ROTATION = 21.64;
    public static final double DEGREES_PER_TICK = 360 / TICKS_PER_ROTATION;
    
  }

  public static class SwerveModuleConstants {

    public int angle_motor_id;
    public int drive_motor_id;
    public int CANCoder_id;
    public Rotation2d angle_offset;

    public SwerveModuleConstants(int angle_motor_id, int drive_motor_id, int CANCoder_id, Rotation2d angle_offset) {

      this.angle_motor_id = angle_motor_id;
      this.drive_motor_id = drive_motor_id;
      this.CANCoder_id = CANCoder_id;
      this.angle_offset = angle_offset;

    }

  }

  public static class SwerveConstants {
    
    public static final boolean angle_invert = false;
    public static final boolean drive_invert = false;

    public static final IdleMode angle_idle_mode = IdleMode.kBrake;
    public static final IdleMode drive_idle_mode = IdleMode.kBrake;

    public static final double angle_kP = 0.1;
    public static final double angle_kI = 0;
    public static final double angle_kD = 0;
    public static final double angle_kFF = 0;

    public static final double drive_kP = 0.1;
    public static final double drive_kI = 0;
    public static final double drive_kD = 0;
    public static final double drive_kFF = 0;

    public static final double max_speed = 4.5;

    public static final double voltage_comp = 12.0;

    public static final double wheel_base = Units.inchesToMeters(1.0);
    public static final double track_width = Units.inchesToMeters(1.0);

    /* Swerve Profiling Values */
    public static final double maxSpeed = 4.5; // meters per second
    public static final double maxAngularVelocity = 11.5;

    public static final SwerveDriveKinematics swerve_kinematics = new SwerveDriveKinematics(
      new Translation2d(wheel_base / 2.0, track_width / 2.0),
      new Translation2d(wheel_base / 2.0, -track_width / 2.0),
      new Translation2d(-wheel_base / 2.0, track_width / 2.0),
      new Translation2d(-wheel_base / 2.0, -track_width / 2.0)
    );

    // ADD IN CANCODER IDS!!!
    public static final SwerveModuleConstants[] module_constants = new SwerveModuleConstants[]{
      new SwerveModuleConstants(DriveConstants.front_left_steer_id, DriveConstants.front_left_drive_id, DriveConstants.front_left_CANcoder_id,  Rotation2d.fromDegrees(0.0)),
      new SwerveModuleConstants(DriveConstants.front_right_steer_id, DriveConstants.front_right_drive_id, DriveConstants.front_right_CANcoder_id,  Rotation2d.fromDegrees(0.0)),
      new SwerveModuleConstants(DriveConstants.rear_left_steer_id, DriveConstants.rear_left_drive_id, DriveConstants.rear_left_CANcoder_id,  Rotation2d.fromDegrees(0.0)),
      new SwerveModuleConstants(DriveConstants.rear_right_steer_id, DriveConstants.rear_right_drive_id, DriveConstants.rear_right_CANcoder_id, Rotation2d.fromDegrees(0.0)),
    };

  }

  public static class ControllerConstants {

    public static final int driver_controller_id = 0;
    public static final double stickDeadband = 0.1;

  }

  public static SwerveModuleState optimize(SwerveModuleState desired_state, Rotation2d current_angle) {

    double target_angle = wrap_angle(current_angle.getDegrees(), desired_state.angle.getDegrees());
    double target_speed = desired_state.speedMetersPerSecond;

    double delta = target_angle - current_angle.getDegrees();

    if (Math.abs(delta) > 90) {
      target_speed = -target_speed;
      target_angle = (delta > 90) ? (target_angle - 180) : (target_angle + 180);
    }

    return new SwerveModuleState(target_speed, Rotation2d.fromDegrees(target_angle));

  }

  public static double wrap_angle(double scope_reference, double angle) {

    double lower_bound;
    double upper_bound;
    double lower_offset = scope_reference % 360;

    if (lower_offset >= 0) {
      lower_bound = scope_reference - lower_offset;
      upper_bound = scope_reference + (360 - lower_offset);
    } else {
      upper_bound = scope_reference - lower_offset;
      lower_bound = scope_reference - (360 + lower_offset);
    }

    while (angle < lower_bound) {
      angle += 360;
    }

    while (angle > upper_bound) {
      angle -= 360;
    }

    if (angle - scope_reference > 180) {
      angle -= 360;
    } else if (angle - scope_reference < -180) {
      angle += 360;
    }

    return angle;

  }

}