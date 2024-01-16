// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// FIRST imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.geometry.Rotation2d;

// REV imports

// CTRE imports
import com.ctre.phoenix.sensors.PigeonIMU;

// Custom imports
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;

public class SwerveDrive extends SubsystemBase {

  private SwerveModule[] swerve_modules;
  private SwerveModulePosition[] swerve_module_positions;

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

  }

  public void zero_imu() {

    this.imu.setYaw(0);

  }

  public Rotation2d get_yaw() {

    return Rotation2d.fromDegrees(this.imu.getYaw());

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
