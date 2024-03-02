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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// CTRE imports
// import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SimSwerveDrivetrain.SimSwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.pathplanner.lib.auto.AutoBuilder;

// Custom imports
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveModule;
import frc.robot.Constants;

public class SwerveDrive extends SubsystemBase {

  private SwerveModule[] swerve_modules;
  private SwerveModulePosition[] swerve_module_positions;
  private Field2d field;

  private final Pigeon2 imu;

  private SwerveDriveOdometry swerve_odometry;

  public SwerveDrive() {

    this.imu = new Pigeon2(DriveConstants.pigeon_id);
    this.zero_imu();

    this.swerve_module_positions = new SwerveModulePosition[]{
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition()
    };

    this.swerve_odometry = new SwerveDriveOdometry(SwerveConstants.swerve_kinematics, this.get_yaw(), this.swerve_module_positions);

    // DON'T MIX UP NUMBERS IT IS CURRENTLY RIGHT IN SHUFFLEBOARD
    this.swerve_modules = new SwerveModule[]{
      new SwerveModule(0, SwerveConstants.module_constants[0]),
      new SwerveModule(1, SwerveConstants.module_constants[1]),
      new SwerveModule(2, SwerveConstants.module_constants[2]),
      new SwerveModule(3, SwerveConstants.module_constants[3])
    };

    Shuffleboard.getTab("Game").addDouble(
      "Pigeon Angle", () -> this.get_yaw().getDegrees()
    );

    for (SwerveModule mod : this.swerve_modules) {

      Shuffleboard.getTab("Game").addDouble(
        "Mod" + mod.module_number + " Angle", () -> mod.getState().angle.getDegrees()
      );

      Shuffleboard.getTab("Game").addDouble(
        "Mod" + mod.module_number + " Speed", () -> mod.getState().speedMetersPerSecond
      );

      Shuffleboard.getTab("Game").addDouble(
        "Mod" + mod.module_number + "CANAngle", () -> mod.getCANDouble()
      );

    }

  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxSpeed);

    for (SwerveModule mod : this.swerve_modules) {
      mod.setDesiredState(desiredStates[mod.module_number], false);
    }
  }

  public Pose2d getPose() {
    return swerve_odometry.getPoseMeters();
  }

  public void zero_imu() {

    this.imu.setYaw(0);

  }

  public void resetToAbsolute() {
    for (int i = 0; i < 4; i++) {
      this.swerve_modules[i].resetToAbsolute();
    }
  }

  public Rotation2d get_yaw() {
    return (DriveConstants.invert_imu)
      ? Rotation2d.fromDegrees(360 - imu.getYaw().getValueAsDouble())
      : Rotation2d.fromDegrees(imu.getYaw().getValueAsDouble());
  }

  public void resetOdometry(Pose2d pose) {
    swerve_odometry.resetPosition(get_yaw(), swerve_module_positions, pose);
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

  class SimSwerveModule {
      private SwerveModulePosition currentPosition = new SwerveModulePosition();
      private SwerveModuleState currentState = new SwerveModuleState();
  
      public SwerveModulePosition getPosition() {
        return currentPosition;
      }
  
      public SwerveModuleState getState() {
        return currentState;
      }
  
    }

}
