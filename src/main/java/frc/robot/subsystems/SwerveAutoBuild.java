// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveDrive;

/**
 * Basic simulation of a swerve subsystem with the methods needed by PathPlanner
 */
public class SwerveAutoBuild extends SubsystemBase {
  private SwerveModule[] modules;
  private SwerveDriveKinematics kinematics;
  private SwerveDriveOdometry odometry;

  private Pigeon2 gyro;
  
  private Field2d field = new Field2d();
  
  public SwerveAutoBuild() {
    gyro = new Pigeon2(DriveConstants.pigeon_id);
    modules = new SwerveModule[]{
        new SwerveModule(0, SwerveConstants.module_constants[0]),
        new SwerveModule(1, SwerveConstants.module_constants[1]),
        new SwerveModule(2, SwerveConstants.module_constants[2]),
        new SwerveModule(3, SwerveConstants.module_constants[3])
    };

    kinematics = new SwerveDriveKinematics(
      SwerveConstants.flModuleOffset, 
      SwerveConstants.frModuleOffset, 
      SwerveConstants.blModuleOffset, 
      SwerveConstants.brModuleOffset
    );

    odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), getPositions());

    // Configure AutoBuilder
    AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }

  @Override
  public void periodic() {
    // Update the simulated gyro, not needed in a real project
    odometry.update(gyro.getRotation2d(), getPositions());

    field.setRobotPose(getPose());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    odometry.resetPosition(gyro.getRotation2d(), getPositions(), pose);
  }

  public ChassisSpeeds getSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
    setStates(targetStates);
  }

  public void setTargetState(SwerveModuleState targetState) {
      // Optimize the state
      currentState = SwerveModuleState.optimize(targetState, currentState.angle);

      currentPosition = new SwerveModulePosition(currentPosition.distanceMeters + (currentState.speedMetersPerSecond * 0.02), currentState.angle);
    }

  public void setStates(SwerveModuleState[] targetStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, SwerveConstants.maxSpeed);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setTargetState(targetStates[i]);
    }
  }

  private SwerveModulePosition currentPosition = new SwerveModulePosition();
    private SwerveModuleState currentState = new SwerveModuleState();

    public SwerveModulePosition getPosition() {
      return currentPosition;
    }

    public SwerveModuleState getState() {
      return currentState;
    }

    public SwerveModuleState[] getModuleStates() {
      SwerveModuleState[] states = new SwerveModuleState[modules.length];
      for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
      }
      return states;
    }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }
    private Rotation2d currentRotation = new Rotation2d();

    public Rotation2d getRotation2d() {
      return currentRotation;
    }

    public void updateRotation(double angularVelRps){
      currentRotation = currentRotation.plus(new Rotation2d(angularVelRps * 0.02));
    }
}