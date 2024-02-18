// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

public class Robot extends TimedRobot {

    private CANSparkMax frontLeftMotor;
    private CANSparkMax frontRightMotor;
    private CANSparkMax rearLeftMotor;
    private CANSparkMax rearRightMotor;

    private DifferentialDrive differentialDrive;
    private XboxController driverController;

    @Override
    public void robotInit() {
        // Initialize CANSparkMax motors with device IDs
        frontLeftMotor = new CANSparkMax(1, CANSparkMax.MotorType.kBrushless);
        frontRightMotor = new CANSparkMax(15, CANSparkMax.MotorType.kBrushless);
        rearLeftMotor = new CANSparkMax(2, CANSparkMax.MotorType.kBrushless);
        rearRightMotor = new CANSparkMax(13, CANSparkMax.MotorType.kBrushless);

        rearLeftMotor.follow(frontLeftMotor);
        rearRightMotor.follow(frontRightMotor);

        // Create a DifferentialDrive object for easier drivetrain control
        differentialDrive = new DifferentialDrive(frontLeftMotor, rearRightMotor);

        // Create a new XboxController object with the driver's controller port
        driverController = new XboxController(0);
    }

    @Override
    public void teleopPeriodic() {
        // Drive the robot with arcade drive using joystick inputs
        double forwardSpeed = -driverController.getLeftY();
        double rotationSpeed = driverController.getRightX();

        differentialDrive.arcadeDrive(forwardSpeed, rotationSpeed);
    }
}
