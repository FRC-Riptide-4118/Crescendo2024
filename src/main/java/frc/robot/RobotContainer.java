// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Java imports
import java.util.*;

// FIRST imports
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

// Pathplanner
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.CommandUtil;

// Subsystems
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.DriveConstants.MotorPosition;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.ExampleAuto;

// Commands
import frc.robot.commands.TeleOpSwerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final SwerveDrive s_SwerveDrive = new SwerveDrive();
  private final Climber         s_Climber = new Climber();
  private final Intake          s_Intake  = new Intake();

  private final SendableChooser<Command> autoChooser;

  /* Commands */

  // Reset to Absolute
  InstantCommand reset_to_abs = new InstantCommand(() -> {this.s_SwerveDrive.resetToAbsolute();}, this.s_SwerveDrive);

  // Left Climber
  InstantCommand run_left_climber_up   = new InstantCommand(() -> {this.s_Climber.LeftRun(0.25); }, this.s_Climber);
  InstantCommand run_left_climber_down = new InstantCommand(() -> {this.s_Climber.LeftRun(-0.25); }, this.s_Climber);

  // Right Climber
  InstantCommand run_right_climber_up   = new InstantCommand(() -> {this.s_Climber.RightRun(0.25); }, this.s_Climber);
  InstantCommand run_right_climber_down = new InstantCommand(() -> {this.s_Climber.RightRun(-0.25); }, this.s_Climber);

  // // Intake
  // InstantCommand intake  = new InstantCommand(() -> {this.s_Intake.RunIntake(-this.driverController.getLeftTriggerAxis());}, this.s_Intake);
  // InstantCommand outtake = new InstantCommand(() -> {this.s_Intake.RunIntake(this.driverController.getRightTriggerAxis());}, this.s_Intake);

  // Xbox Controller
  private final CommandXboxController driverController =
      new CommandXboxController(ControllerConstants.driver_controller_id);

  private final Joystick driver = new Joystick(0);

  // POV Button
  // private final POVButton povButton =
  //     new POVButton(new GenericHID(0), 0);

  // IMU 
  // private final JoystickButton zero_imu = 
  //   new JoystickButton(driver, XboxController.Button.kY.value);

  // 
  // private final JoystickButton robotCentric =
  //   new JoystickButton(driver, XboxController.Button.kA.value);


  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {

    s_SwerveDrive.setDefaultCommand(
      new TeleOpSwerve(
      s_SwerveDrive, 
      () -> driverController.getLeftY(), 
      () -> -driverController.getLeftX(), 
      () -> -driverController.getRightX(),
      () -> driverController.back().getAsBoolean())
    );

    // s_Intake.setDefaultCommand(intake);
    // s_Intake.setDefaultCommand(outtake);

    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);
  }


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    SmartDashboard.putData("Leave Auto", new PathPlannerAuto("Leave Auto"));
    SmartDashboard.putData("Spin Auto", new PathPlannerAuto("Spin Auto"));

    // Zeroing the Modules
    driverController.povUp().whileTrue(reset_to_abs);

    // Zeroing the IMU
    driverController.start().whileTrue(new InstantCommand(()-> s_SwerveDrive.zero_imu()));

    // Intake // TESTING FOR SLIDES
    driverController.a().onTrue(new InstantCommand(() -> s_Intake.Intake())).onFalse(new InstantCommand(() -> {s_Intake.Off(); }, s_Intake));
    driverController.b().onTrue(new InstantCommand(() -> s_Intake.Outtake())).onFalse(new InstantCommand(() -> {s_Intake.Off(); }, s_Intake));

    // Left Climber
    driverController.leftBumper().onTrue(run_left_climber_up).onFalse(new InstantCommand(() -> {s_Climber.LeftRun(0); }, s_Climber));
    driverController.leftStick().onTrue(run_left_climber_down).onFalse(new InstantCommand(() -> {s_Climber.LeftRun(0); }, s_Climber));

    // Right Climber
    driverController.rightBumper().onTrue(run_right_climber_up).onFalse(new InstantCommand(() -> {s_Climber.RightRun(0); }, s_Climber));
    driverController.rightStick().onTrue(run_right_climber_down).onFalse(new InstantCommand(() -> {s_Climber.RightRun(0); }, s_Climber));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return autoChooser.getSelected();

  }
}
