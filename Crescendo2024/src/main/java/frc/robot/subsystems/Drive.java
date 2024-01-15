// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
// FIRST imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

// REV imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

public class Drive extends SubsystemBase {
  
  private CANSparkMax front_left_drive; 
  private CANSparkMax front_right_drive; 
  private CANSparkMax rear_left_drive; 
  private CANSparkMax rear_right_drive; 

  private CANSparkMax front_left_steer; 
  private CANSparkMax front_right_steer; 
  private CANSparkMax rear_left_steer; 
  private CANSparkMax rear_right_steer; 

  private RelativeEncoder front_left_drive_encoder;
  private RelativeEncoder front_right_drive_encoder;
  private RelativeEncoder rear_left_drive_encoder;
  private RelativeEncoder rear_right_drive_encoder;

  private RelativeEncoder front_left_steer_encoder;
  private RelativeEncoder front_right_steer_encoder;
  private RelativeEncoder rear_left_steer_encoder;
  private RelativeEncoder rear_right_steer_encoder;

  private SparkPIDController front_left_steer_PIDController;
  private SparkPIDController front_right_steer_PIDController;
  private SparkPIDController rear_left_steer_PIDController;
  private SparkPIDController rear_right_steer_PIDController;

  /** Creates a new Drive. */
  public Drive() {

    this.front_left_drive  = new CANSparkMax(DriveConstants.front_left_drive_id, MotorType.kBrushless);
    this.front_right_drive = new CANSparkMax(DriveConstants.front_right_drive_id, MotorType.kBrushless);
    this.rear_left_drive   = new CANSparkMax(DriveConstants.rear_left_drive_id, MotorType.kBrushless);
    this.rear_right_drive  = new CANSparkMax(DriveConstants.rear_right_drive_id, MotorType.kBrushless);

    this.front_left_steer  = new CANSparkMax(DriveConstants.front_left_steer_id, MotorType.kBrushless);
    this.front_right_steer = new CANSparkMax(DriveConstants.front_right_steer_id, MotorType.kBrushless);
    this.rear_left_steer   = new CANSparkMax(DriveConstants.rear_left_steer_id, MotorType.kBrushless);
    this.rear_right_steer  = new CANSparkMax(DriveConstants.rear_right_steer_id, MotorType.kBrushless);

    this.front_left_drive_encoder = this.front_left_drive.getEncoder();
    this.front_right_drive_encoder = this.front_right_drive.getEncoder();
    this.rear_left_drive_encoder = this.rear_left_drive.getEncoder();
    this.rear_right_drive_encoder = this.rear_right_drive.getEncoder();

    this.front_left_steer_encoder = this.front_left_steer.getEncoder();
    this.front_right_steer_encoder = this.front_right_steer.getEncoder();
    this.rear_left_steer_encoder = this.rear_left_steer.getEncoder();
    this.rear_right_steer_encoder = this.rear_right_steer.getEncoder();

    this.front_left_steer_PIDController = this.front_left_steer.getPIDController();
    this.front_right_steer_PIDController = this.front_right_steer.getPIDController();
    this.rear_left_steer_PIDController = this.rear_left_steer.getPIDController();
    this.rear_right_steer_PIDController = this.rear_right_steer.getPIDController();

    this.front_left_steer_PIDController.setP(Constants.DriveConstants.steer_kP);
    this.front_right_steer_PIDController.setP(Constants.DriveConstants.steer_kP);
    this.rear_left_steer_PIDController.setP(Constants.DriveConstants.steer_kP);
    this.rear_right_steer_PIDController.setP(Constants.DriveConstants.steer_kP);

    this.front_left_steer_PIDController.setI(Constants.DriveConstants.steer_kI);
    this.front_right_steer_PIDController.setI(Constants.DriveConstants.steer_kI);
    this.rear_left_steer_PIDController.setI(Constants.DriveConstants.steer_kI);
    this.rear_right_steer_PIDController.setI(Constants.DriveConstants.steer_kI);

    this.front_left_steer_PIDController.setD(Constants.DriveConstants.steer_kD);
    this.front_right_steer_PIDController.setD(Constants.DriveConstants.steer_kD);
    this.rear_left_steer_PIDController.setD(Constants.DriveConstants.steer_kD);
    this.rear_right_steer_PIDController.setD(Constants.DriveConstants.steer_kD);
  
  }

  private CANSparkMax get_motor_controller(Constants.DriveConstants.MotorPosition motor) {

    switch(motor) {

      case C_FRONT_LEFT_DRIVE : {
        return this.front_left_drive;
      }
      case C_FRONT_RIGHT_DRIVE : {
        return this.front_right_drive;
      }
      case C_REAR_LEFT_DRIVE : {
        return this.rear_left_drive;
      }
      case C_REAR_RIGHT_DRIVE : {
        return this.rear_right_drive;
      }
      case C_FRONT_LEFT_STEER : {
        return this.front_left_steer;
      }
      case C_FRONT_RIGHT_STEER : {
        return this.front_right_steer;
      }
      case C_REAR_LEFT_STEER : {
        return this.rear_left_steer;
      }
      case C_REAR_RIGHT_STEER : {
        return this.rear_right_steer;
      }
      default : {
        return null;
      }

    }

  }

  private RelativeEncoder get_encoder(Constants.DriveConstants.MotorPosition motor) {

    switch(motor) {

      case C_FRONT_LEFT_DRIVE : {
        return this.front_left_drive_encoder;
      }
      case C_FRONT_RIGHT_DRIVE : {
        return this.front_right_drive_encoder;
      }
      case C_REAR_LEFT_DRIVE : {
        return this.rear_left_drive_encoder;
      }
      case C_REAR_RIGHT_DRIVE : {
        return this.rear_right_drive_encoder;
      }
      case C_FRONT_LEFT_STEER : {
        return this.front_left_steer_encoder;
      }
      case C_FRONT_RIGHT_STEER : {
        return this.front_right_steer_encoder;
      }
      case C_REAR_LEFT_STEER : {
        return this.rear_left_steer_encoder;
      }
      case C_REAR_RIGHT_STEER : {
        return this.rear_right_steer_encoder;
      }
      default : {
        return null;
      }

    }

  }
  
  private SparkPIDController get_PIDController(Constants.DriveConstants.MotorPosition motor) {

    switch(motor) {

      case C_FRONT_LEFT_DRIVE : {
        return null;
      }
      case C_FRONT_RIGHT_DRIVE : {
        return null;
      }
      case C_REAR_LEFT_DRIVE : {
        return null;
      }
      case C_REAR_RIGHT_DRIVE : {
        return null;
      }
      case C_FRONT_LEFT_STEER : {
        return this.front_left_steer_PIDController;
      }
      case C_FRONT_RIGHT_STEER : {
        return this.front_right_steer_PIDController;
      }
      case C_REAR_LEFT_STEER : {
        return this.rear_left_steer_PIDController;
      }
      case C_REAR_RIGHT_STEER : {
        return this.rear_right_steer_PIDController;
      }
      default : {
        return null;
      }

    }

  }

  public void drive_motor(Constants.DriveConstants.MotorPosition motor, double speed) {

    CANSparkMax motor_controller = this.get_motor_controller(motor);
    if (motor_controller != null)
      motor_controller.set(speed);

  }
 
  public void reset_encoder_position(Constants.DriveConstants.MotorPosition motor) {

    RelativeEncoder encoder = this.get_encoder(motor);
    if (encoder != null)
      encoder.setPosition(0.0);

  }

  public void reset_ALL_encoder_position() {

    reset_encoder_position(DriveConstants.MotorPosition.C_FRONT_LEFT_STEER);
    reset_encoder_position(DriveConstants.MotorPosition.C_FRONT_RIGHT_STEER);
    reset_encoder_position(DriveConstants.MotorPosition.C_REAR_LEFT_STEER);
    reset_encoder_position(DriveConstants.MotorPosition.C_REAR_RIGHT_STEER);

  }

  public double get_integrated_encoder_position(Constants.DriveConstants.MotorPosition motor) {

    RelativeEncoder encoder = this.get_encoder(motor);
    if (encoder != null)
      return encoder.getPosition();
    return 0;

  }

  public double get_integrated_encoder_angle(Constants.DriveConstants.MotorPosition motor) {

    double ticks = this.get_integrated_encoder_position(motor);
    double angle = ticks * DriveConstants.DEGREES_PER_TICK;
    if (angle < 0)
      return 360 - angle;
    return angle;

  }

  public double get_encoder_position_from_angle(double angle) {

    return angle / DriveConstants.DEGREES_PER_TICK;

  }

  public void turn_wheel_to_angle(DriveConstants.MotorPosition motor, double angle) {
    
    SparkPIDController pid_controller = this.get_PIDController(motor);
    pid_controller.setReference(this.get_encoder_position_from_angle(angle), CANSparkBase.ControlType.kPosition);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
