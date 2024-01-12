package frc.robot.Commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;

public class WheelDrive{
    
    private CANSparkMax angleMotor;
    private CANSparkMax speedMotor;
    // private SparkPIDController pidController;

    public WheelDrive(int angleMotor_id, int speedMotor_id, int encoder) {

        this.angleMotor     = new CANSparkMax(angleMotor_id, MotorType.kBrushless);
        this.speedMotor     = new CANSparkMax(speedMotor_id, MotorType.kBrushless);
        // this.pidController  = this.angleMotor.getPIDController();
        // this.pidController  = this.speedMotor.getPIDController();

        // pidController.setOutputRange( -1, 1);
        // pidController.
    }

    public void drive (double speed, double angle) {

        this.speedMotor.set(speed);

    }
}
