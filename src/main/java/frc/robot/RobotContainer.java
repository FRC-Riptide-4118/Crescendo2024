// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Java imports
import java.util.*;

// FIRST imports
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;


// Subsystems
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Climber;
import frc.robot.Constants.DriveConstants.MotorPosition;

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
  private final Climber s_Climber = new Climber();

  // Commands
  InstantCommand do_nothing = new InstantCommand( () -> {} );
  RunCommand run_climber = new RunCommand(() -> { this.s_Climber.Run(this.driverController.getRightY()); }, this.s_Climber);
  // RunCommand reset_encoders = new RunCommand(() -> {this.s_SwerveDrive.}, null)

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(Constants.ControllerConstants.driver_controller_id);

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {

    // Shuffleboard

    s_SwerveDrive.setDefaultCommand(
      new TeleOpSwerve(
      s_SwerveDrive, 
      () -> driverController.getLeftY(), 
      () -> driverController.getLeftX(), 
      () -> driverController.getRightX(),
      () -> true)
    );

    // s_Climber.setDefaultCommand( run_climber );

    // Configure the trigger bindings
    configureBindings();
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

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return this.do_nothing;

  }
}
