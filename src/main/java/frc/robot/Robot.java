// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Photon Vision & Pathplanner
import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveControlRequestParameters;
import com.pathplanner.lib.commands.PathPlannerAuto;

// WPI Imports
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

// Custom Imports
import frc.robot.Vision.Limelight;
import frc.robot.Vision.LimelightHelpers;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveDrive;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  // private SwerveDrive s_Swerve;

  // private final Limelight limelight = new Limelight();

  //NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  // NetworkTable limelightAprilTable = NetworkTableInstance.getDefault().getTable("limelight-april");
  // NetworkTable limelightNoteTable = NetworkTableInstance.getDefault().getTable("limelight-note");

  // double limelightAprilTagLastError;
  // double limelightNoteLastError;

  // PhotonCamera photon = new PhotonCamera("Microsoft_LifeCam_HD-3000");

  //Photon Vision PID Setup
  // final double ANGULAR_P = 0.1;
  // final double ANGULAR_D = 0.0;
  // PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // limelight.updateLimelightData();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  // private void Driver1Controls() {
  //     if (ControllerConstants.driver1.getRawButton(3)) {
  //       limelightAprilTagAim(true);
  //     } else if (ControllerConstants.driver1.getRawButton(4)) {
  //       limelightNoteAim(true);

  //     if (!(ControllerConstants.driver1.getRawButton(3))) {
  //       limelightAprilTagLastError = 0;
  //       System.out.println("limelight button not pressed, setting last tx to 0");
  //     }
  //     if (!(ControllerConstants.driver1.getRawButton(4))) {
  //       limelightNoteLastError = 0;
  //       System.out.println("limelight button not pressed, setting last tx to 0");
  //     }
  //   }
  // }

  // private void RobotTelemetry() {

  //   NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  //   NetworkTableEntry tx = table.getEntry("tx");
  //   NetworkTableEntry ty = table.getEntry("ty");
  //   NetworkTableEntry ta = table.getEntry("ta");

  //   double x = tx.getDouble(0.0);
  //   double y = ty.getDouble(0.0);
  //   double area = ta.getDouble(0.0);

  //   SmartDashboard.putNumber("LimelightX", x);
  //   SmartDashboard.putNumber("LimelightY", y);
  //   SmartDashboard.putNumber("LimelightArea", area);
  // }

  // To Drive With Controllers
  // private void SwerveDrive(boolean isFieldRel) {
  //   // Controller Deadbands (Translation, Strafe, Rotation)

  //   double xSpeed = MathUtil.applyDeadband(Constants.ControllerConstants.driver1.getRawAxis(1)
  //       * ((Constants.ControllerConstants.driver1.getRawAxis(2) + 1) / 2),
  //       Constants.ControllerConstants.stickDeadband);
  //   double ySpeed = MathUtil.applyDeadband(ControllerConstants.driver1.getRawAxis(0)
  //       * ((ControllerConstants.driver1.getRawAxis(2) + 1) / 2),
  //       ControllerConstants.stickDeadband);
  //   double rot = MathUtil.applyDeadband(-ControllerConstants.driver1.getRawAxis(3)
  //       * ((ControllerConstants.driver1.getRawAxis(2) + 1) / 2),
  //       ControllerConstants.stickDeadband);

  //   if (ControllerConstants.driver1.getPOV() == 0) {
  //     xSpeed = -.75;
  //   } else if (ControllerConstants.driver1.getPOV() == 90) {
  //     ySpeed = .75;
  //   } else if (ControllerConstants.driver1.getPOV() == 180) {
  //     xSpeed = .75;
  //   } else if (ControllerConstants.driver1.getPOV() == 270) {
  //     ySpeed = -.75;
  //   } else if (ControllerConstants.driver1.getPOV() == 45) {
  //     xSpeed = .5;
  //     ySpeed = .5;
  //   } else if (ControllerConstants.driver1.getPOV() == 135) {
  //     xSpeed = -.5;
  //     ySpeed = .5;
  //   } else if (ControllerConstants.driver1.getPOV() == 225) {
  //     xSpeed = -.5;
  //     ySpeed = -.5;
  //   } else if (ControllerConstants.driver1.getPOV() == 315) {
  //     xSpeed = .5;
  //     ySpeed = -.5;
  //   }

  //   // Drive Function
  //   s_Swerve.drive(new Translation2d(xSpeed, ySpeed).times(SwerveConstants.maxSpeed),
  //       rot * SwerveConstants.maxAngularVelocity, isFieldRel, false);
  // }

  // private void limelightAprilTagAim(boolean isFieldRel) {
  //   double currentGyro = s_Swerve.imu.getYaw();
  //   double mappedAngle = 0.0f;
  //   double angy = ((currentGyro % 360.0f));
  //   if (currentGyro >= 0.0f) {
  //     if (angy > 180) {
  //       mappedAngle = angy - 360.0f;
  //     } else {
  //       mappedAngle = angy;
  //     }
  //   } else {
  //     if (Math.abs(angy) > 180.0f) {
  //       mappedAngle = angy + 360.0f;
  //     } else {
  //       mappedAngle = angy;
  //     }
  //   }
  //   double tx = limelightAprilTable.getEntry("tx").getFloat(700);
  //   System.out.println("tx april: " + tx);
  //   double tx_max = 30.0f; // detemined empirically as the limelights field of view
  //   double error = 0.0f;
  //   double kP = 2.0f; // should be between 0 and 1, but can be greater than 1 to go even faster
  //   double kD = 0.0f; // should be between 0 and 1
  //   double steering_adjust = 0.0f;
  //   double acceptable_error_threshold = 10.0f / 360.0f; // 15 degrees allowable
  //   if (tx != 0.0f) { // use the limelight if it recognizes anything, and use the gyro otherwise
  //     error = -1.0f * (tx / tx_max) * (31.65 / 180); // scaling error between -1 and 1, with 0 being dead on, and 1
  //                                                    // being 180 degrees away
  //   } else {
  //     error = mappedAngle / 180.0f; // scaling error between -1 and 1, with 0 being dead on, and 1 being 180 degrees
  //                                   // away
  //   }
  //   if (limelightAprilTagLastError == 0.0f) {
  //     limelightAprilTagLastError = tx;
  //   }
  //   double error_derivative = error - limelightAprilTagLastError;
  //   limelightAprilTagLastError = tx; // setting limelightlasterror for next loop

  //   if (Math.abs(error) > acceptable_error_threshold) { // PID with a setpoint threshold
  //     steering_adjust = (kP * error + kD * error_derivative);
  //   }

  //   final double xSpeed = MathUtil.applyDeadband(ControllerConstants.driver1.getRawAxis(1),
  //       ControllerConstants.stickDeadband);
  //   final double ySpeed = MathUtil.applyDeadband(ControllerConstants.driver1.getRawAxis(0),
  //       ControllerConstants.stickDeadband);
  //   s_Swerve.drive(new Translation2d(xSpeed, ySpeed).times(SwerveConstants.maxSpeed),
  //       steering_adjust * SwerveConstants.maxAngularVelocity, isFieldRel, false);

  //   System.out.println("raw angle: " + currentGyro + ", mapped angle: " + mappedAngle + ", april tag error: " + error);
  // }

// private void limelightNoteAim(boolean isFieldRel) {
//     double tx = limelightNoteTable.getEntry("tx").getFloat(0);
//     double tx_max = 30.0f; // detemined empirically as the limelights field of view
//     double error = 0.0f;
//     double kP = 2.0f; // should be between 0 and 1, but can be greater than 1 to go even faster
//     double kD = 0.0f; // should be between 0 and 1
//     double steering_adjust = 0.0f;
//     double acceptable_error_threshold = 10.0f / 360.0f; // 15 degrees allowable
//     error = -1.0 * (tx / tx_max) * (31.65 / 180); // scaling error between -1 and 1, with 0 being dead on, and 1 being 180 degrees away
//     if (limelightNoteLastError == 0.0f) {
//       limelightNoteLastError = tx;
//     }
//     double error_derivative = error - limelightNoteLastError;
//     limelightNoteLastError = tx; // setting limelightlasterror for next loop

//     if (Math.abs(error) > acceptable_error_threshold) { // PID with a setpoint threshold
//       steering_adjust = (kP * error + kD * error_derivative);
//     }

//     final double xSpeed = MathUtil.applyDeadband(ControllerConstants.driver1.getRawAxis(1),
//         ControllerConstants.stickDeadband);
//     final double ySpeed = MathUtil.applyDeadband(ControllerConstants.driver1.getRawAxis(0),
//         ControllerConstants.stickDeadband);
//     s_Swerve.drive(new Translation2d(xSpeed, ySpeed).times(SwerveConstants.maxSpeed),
//         steering_adjust * SwerveConstants.maxAngularVelocity, isFieldRel, false);

//     System.out.println("Note error: " + error);
//   }

//   //uses photon vision, we are using limelight, keeping in case the code is useful later
//   public void NoteAutoAim(boolean isFieldRel){
//     double steering_adjust;
//     var result = photon.getLatestResult();

//     if (result.hasTargets()){
//       steering_adjust = -turnController.calculate(result.getBestTarget().getYaw(), 0);
//     } else {
//       steering_adjust = MathUtil.applyDeadband(-ControllerConstants.driver1.getRawAxis(3)
//         * ((ControllerConstants.driver1.getRawAxis(2) + 1) / 2),
//         ControllerConstants.stickDeadband);
//     }

//     final double xSpeed = MathUtil.applyDeadband(ControllerConstants.driver1.getRawAxis(1),
//         ControllerConstants.stickDeadband);
//     final double ySpeed = MathUtil.applyDeadband(ControllerConstants.driver1.getRawAxis(0),
//         ControllerConstants.stickDeadband);

//     s_Swerve.drive(
//         new Translation2d(xSpeed, ySpeed).times(SwerveConstants.maxSpeed),
//         steering_adjust * SwerveConstants.maxAngularVelocity,
//         isFieldRel,
//         true);
//   }
}
