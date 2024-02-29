// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// FIRST imports
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.geometry.Translation2d;

// REV imports
import com.revrobotics.CANSparkBase.IdleMode;

// Java imports
import java.util.*;

// Pheonix imports
import com.ctre.phoenix6.hardware.CANcoder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

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

    // CANCoders
    public static final int front_left_CANcoder_id  = 15;
    public static final int front_right_CANcoder_id = 16;
    public static final int rear_left_CANcoder_id   = 17;
    public static final int rear_right_CANcoder_id  = 18;

    // IMU
    public static final boolean invert_imu = false;

    // Pigeon
    public static final int pigeon_id = 20;

    public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
    public static final double angleGearRatio = (6.75*3 / 1.0); // 12.8:1

    // Math
    public static final double TICKS_PER_ROTATION = 42;
    public static final double DEGREES_PER_TICK = 360 / TICKS_PER_ROTATION;
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;
    
  }

  public static class SwerveModuleConstants {

    public int angle_motor_id;
    public int drive_motor_id;
    public int CANCoder_id;
    public Rotation2d angle_offset;

    public SwerveModuleConstants(int angle_motor_id, int drive_motor_id, int CANCoder_id /*, Rotation2d angle_offset */ ) {

      this.angle_motor_id = angle_motor_id;
      this.drive_motor_id = drive_motor_id;
      this.CANCoder_id = CANCoder_id;
    }

  }

  public static class SwerveConstants {
    
    public static final boolean angle_invert = false;
    public static final boolean drive_invert = false;

    public static final IdleMode angle_idle_mode = IdleMode.kBrake;
    public static final IdleMode drive_idle_mode = IdleMode.kBrake;

    public static final int angle_smart_current_limit = 20;
    public static final int drive_smart_current_limit = 80;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor =
      (DriveConstants.wheelDiameter * Math.PI) / DriveConstants.driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;

    public static final double angleConversionFactor = 360/ DriveConstants.angleGearRatio;

    public static final double angle_kP  = 0.1;
    public static final double angle_kI  = 0.001;
    public static final double angle_kD  = 0.0;
    public static final double angle_kFF = 0.0;

    public static final double drive_kP  = 0.1;
    public static final double drive_kI  = 0.0;
    public static final double drive_kD  = 0.0;
    public static final double drive_kFF = 0.0;

    public static final double max_speed = 4.5;

    public static final double voltage_comp = 12.0;

    public static final double wheel_base = Units.inchesToMeters(1.0);
    public static final double track_width = Units.inchesToMeters(1.0);

    /* Swerve Profiling Values */
    public static final double maxSpeed = 4.5; // meters per second
    public static final double maxAngularVelocity = 11.5; // 11.5

    public static final SwerveDriveKinematics swerve_kinematics = new SwerveDriveKinematics(
      new Translation2d(wheel_base / 2.0, -track_width / 2.0),  // +-
      new Translation2d(wheel_base / 2.0, track_width / 2.0),   // ++
      new Translation2d(-wheel_base / 2.0, -track_width / 2.0), // -- 
      new Translation2d(-wheel_base / 2.0, +track_width / 2.0)  // -+
    );

    // Took out angle offsets
    public static final SwerveModuleConstants[] module_constants = new SwerveModuleConstants[]{
      new SwerveModuleConstants(DriveConstants.front_left_steer_id, DriveConstants.front_left_drive_id, DriveConstants.front_left_CANcoder_id),
      new SwerveModuleConstants(DriveConstants.front_right_steer_id, DriveConstants.front_right_drive_id, DriveConstants.front_right_CANcoder_id),
      new SwerveModuleConstants(DriveConstants.rear_left_steer_id, DriveConstants.rear_left_drive_id, DriveConstants.rear_left_CANcoder_id),
      new SwerveModuleConstants(DriveConstants.rear_right_steer_id, DriveConstants.rear_right_drive_id, DriveConstants.rear_right_CANcoder_id),
    };
    
    public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
      new PIDConstants(15, 0, 0), // Translation constants 
      new PIDConstants(30, 0, 0), // Rotation constants 
      4.5, 
      25.114, // Drive base radius (distance from center to furthest module) 
      new ReplanningConfig()
    );
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

  public static class ControllerConstants {

    public static final int driver_controller_id = 0;
    public static final double stickDeadband = 0.1;

    public static final Joystick driver1 = new Joystick(0);

  }

  public static class IntakeConstants {
    public static final int intake_id = 10;
  }

  public static class ClimberConstants {
    public static final int left_climber_id = 9;
    public static final int right_climber_id = 14;

    public static final int left_climber_current_limit  = 40;
    public static final int right_climber_current_limit = 40;
  }

  public static class LauncherConstants {
    public static final int launcher_id = 12;

    public static final double launcher_kp = 0.0;
    public static final double launcher_ki = 0.0;
    public static final double launcher_kd = 0.0;

  }

  public static class SlidesConstants {
    public static final int slides_id = 13;

    // Slides
    public static final double slides_kp = 0.1;
    public static final double slides_ki = 0.0;
    public static final double slides_kd = 0.0;

    // Positions
    public static final double position1 = 0;
    public static final double position2 = 2;
    public static final double position3 = 4;
    public static final double tolerance = 1;

  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond =1.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.5;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

}
