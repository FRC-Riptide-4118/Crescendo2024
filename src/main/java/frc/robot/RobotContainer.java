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

// Subsystems
import frc.robot.subsystems.Drive;
import frc.robot.Constants.DriveConstants.MotorPosition;

// Commands
import frc.robot.commands.Reset;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final Drive drive = new Drive(); 

  // Commands
  InstantCommand do_nothing = new InstantCommand( () -> {} );
  
  RunCommand turn_test = new RunCommand(
    () -> {this.drive.turn_wheel_to_angle(MotorPosition.C_FRONT_RIGHT_STEER, 90);}, 
    drive);
  
    Reset reset_command = new Reset(this.drive);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(Constants.ControllerConstants.driver_controller_id);

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {

    // Shuffleboard
    Shuffleboard.getTab("Game").addDouble("ENCODER", 
    () -> {return this.drive.get_integrated_encoder_angle(MotorPosition.C_FRONT_RIGHT_STEER); });

    Shuffleboard.getTab("Game").addDouble("CONTROLLER ANGLE",
    () -> {return 180+Constants.DEG_PER_RAD*Math.atan2(this.driverController.getLeftY(), this.driverController.getLeftX()); });

    SmartDashboard.putData("RESET", reset_command); 

    // Set Default Commands
    this.drive.setDefaultCommand(

      Commands.run(

        () -> { 

          // this.drive.drive_motor(Constants.DriveConstants.MotorPosition.C_FRONT_LEFT_DRIVE, this.driverController.getLeftY());
          // this.drive.drive_motor(Constants.DriveConstants.MotorPosition.C_FRONT_RIGHT_DRIVE, this.driverController.getLeftY());
          // this.drive.drive_motor(Constants.DriveConstants.MotorPosition.C_REAR_LEFT_DRIVE, this.driverController.getLeftY());
          // this.drive.drive_motor(Constants.DriveConstants.MotorPosition.C_REAR_RIGHT_DRIVE, this.driverController.getLeftY());

          this.drive.turn_wheel_to_angle(MotorPosition.C_FRONT_LEFT_STEER, 180+Constants.DEG_PER_RAD*Math.atan2(this.driverController.getLeftY(), this.driverController.getLeftX()));
          this.drive.turn_wheel_to_angle(MotorPosition.C_FRONT_RIGHT_STEER, 180+Constants.DEG_PER_RAD*Math.atan2(this.driverController.getLeftY(), this.driverController.getLeftX()));
          this.drive.turn_wheel_to_angle(MotorPosition.C_REAR_LEFT_STEER, 180+Constants.DEG_PER_RAD*Math.atan2(this.driverController.getLeftY(), this.driverController.getLeftX()));
          this.drive.turn_wheel_to_angle(MotorPosition.C_REAR_RIGHT_STEER, 180+Constants.DEG_PER_RAD*Math.atan2(this.driverController.getLeftY(), this.driverController.getLeftX()));

        },
        
        this.drive

      )

    );

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

    this.driverController
    .a()
    .whileTrue(
        turn_test
      );

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
