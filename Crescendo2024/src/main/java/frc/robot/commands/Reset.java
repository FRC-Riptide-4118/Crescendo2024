// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// FIRST imports
import edu.wpi.first.wpilibj2.command.InstantCommand;
// Subsystems
import frc.robot.subsystems.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Reset extends InstantCommand {

  private Drive drive;

  public Reset(Drive drive) {

    this.drive = drive;
    addRequirements(this.drive); 

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    this.drive.reset_ALL_encoder_position();

  }
}
