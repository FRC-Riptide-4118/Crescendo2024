// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// FIRST imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

// REV imports
import com.revrobotics.SparkAbsoluteEncoder;

// CTRE imports
import com.ctre.phoenix.sensors.PigeonIMU;

// Custom imports
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.EncoderConstants;
import frc.robot.Constants.SwerveConstants;
// import frc.robot.Constants.;


public class SwerveDrive extends SubsystemBase {

  private SwerveModule[] swerve_modules;
  private SwerveModulePosition[] swerve_module_positions;
  private SparkAbsoluteEncoder[] absolute_encoder;

  private PigeonIMU imu;

  SwerveDriveOdometry swerve_odometry;

  public SwerveDrive() {

    this.imu = new PigeonIMU(DriveConstants.pigeon_id);
    this.imu.configFactoryDefault();
    this.zero_imu();

    this.swerve_module_positions = new SwerveModulePosition[]{
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition()
    };

    this.swerve_odometry = new SwerveDriveOdometry(SwerveConstants.swerve_kinematics, this.get_yaw(), this.swerve_module_positions);

    this.swerve_modules = new SwerveModule[]{
      new SwerveModule(0, SwerveConstants.module_constants[0]),
      new SwerveModule(1, SwerveConstants.module_constants[1]),
      new SwerveModule(2, SwerveConstants.module_constants[2]),
      new SwerveModule(3, SwerveConstants.module_constants[3])
    };

    for (SwerveModule mod : this.swerve_modules) {

      Shuffleboard.getTab("Game").addDouble(
        "Mod" + mod.module_number + " Angle", () -> mod.getState().angle.getDegrees()
      );

      Shuffleboard.getTab("Game").addDouble(
        "Mod" + mod.module_number + " Speed", () -> mod.getState().speedMetersPerSecond
      );

    }

    Shuffleboard.getTab("Game").addDouble
      (
        "ENCODER", 
        get_encoder()
      );

  }

  public SparkAbsoluteEncoder[] get_encoder() {

    this.absolute_encoder = new SparkAbsoluteEncoder[EncoderConstants.front_right_encoder_id];
    return this.absolute_encoder;

  }

  public void zero_imu() {

    this.imu.setYaw(0);

  }

  public Rotation2d get_yaw() {

    return Rotation2d.fromDegrees(this.imu.getYaw());

  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        SwerveConstants.swerve_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, this.get_yaw())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);

    for (SwerveModule mod : this.swerve_modules) {
      mod.setDesiredState(swerveModuleStates[mod.module_number], isOpenLoop);
    }
  }

  @Override
  public void periodic() {

  }
}
