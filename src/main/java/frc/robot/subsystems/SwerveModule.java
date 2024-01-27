package frc.robot.subsystems;

// FIRST imports
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Encoder;

// REV imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.ControlType;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
// Pheonix imports
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

// Custom imports
import frc.robot.Constants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.DriveConstants;

// Java imports
import java.util.*;

public class SwerveModule {
    
    public int module_number;

    private Rotation2d prev_angle;
    private Rotation2d angle_offset;

    private CANSparkMax angle_motor;
    private CANSparkMax drive_motor;

    private CANcoder CANCoder;
    private StatusSignal pos;

    private RelativeEncoder angle_encoder;
    private RelativeEncoder drive_encoder;

    private SparkPIDController angle_controller;
    private SparkPIDController drive_controller;

    public SwerveModule(int module_number, SwerveModuleConstants module_constants) {

        this.module_number = module_number;
        this.angle_offset = module_constants.angle_offset;

        this.angle_motor = new CANSparkMax(module_constants.angle_motor_id, MotorType.kBrushless);
        this.angle_encoder = this.angle_motor.getEncoder();        
        this.angle_controller = this.angle_motor.getPIDController();
        this.configAngleMotor();

        this.drive_motor = new CANSparkMax(module_constants.drive_motor_id, MotorType.kBrushless);
        this.drive_encoder = this.drive_motor.getEncoder();
        this.drive_controller = this.drive_motor.getPIDController();
        this.configDriveMotor();

        this.CANCoder = new CANcoder(module_constants.CANCoder_id);
        this.pos = this.CANCoder.getAbsolutePosition();
        this.configCANcoder();

        this.prev_angle = this.getState().angle;

    }

    public void resetToAbsolute() {
    double absolutePosition = getCANCoder().getDegrees() - angle_offset.getDegrees();
    angle_encoder.setPosition(absolutePosition);
    }

    private void configAngleMotor() {

        this.angle_motor.restoreFactoryDefaults();
        this.angle_motor.setInverted(SwerveConstants.angle_invert);
        this.angle_motor.setIdleMode(SwerveConstants.angle_idle_mode);

        this.angle_encoder.setPositionConversionFactor(DriveConstants.DEGREES_PER_TICK);
        this.angle_encoder.setPosition(0.0);

        this.angle_controller.setP(SwerveConstants.angle_kP);
        this.angle_controller.setI(SwerveConstants.angle_kI);
        this.angle_controller.setD(SwerveConstants.angle_kD);
        this.angle_controller.setFF(SwerveConstants.angle_kFF);

        this.angle_motor.enableVoltageCompensation(SwerveConstants.voltage_comp);
        this.angle_motor.burnFlash();

    }

    private void configCANcoder() {
        CANcoderConfiguration configs = new CANcoderConfiguration();

        configs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        configs.MagnetSensor.MagnetOffset = 0.26;
        configs.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        this.CANCoder.getConfigurator().apply(configs);

    }

    private void configDriveMotor() {

        this.drive_motor.restoreFactoryDefaults();
        this.drive_motor.setInverted(SwerveConstants.drive_invert);
        this.drive_motor.setIdleMode(SwerveConstants.drive_idle_mode);

        // this.drive_encoder.setPositionConversionFactor();
        // this.drive_encoder.setVelocityConversionFactor();

        this.drive_controller.setP(SwerveConstants.drive_kP);
        this.drive_controller.setI(SwerveConstants.drive_kI);
        this.drive_controller.setD(SwerveConstants.drive_kD);
        this.drive_controller.setFF(SwerveConstants.drive_kFF);

        this.angle_motor.enableVoltageCompensation(SwerveConstants.voltage_comp);
        this.angle_motor.burnFlash();
        this.drive_encoder.setPosition(0.0);

    }

    public void setDesiredState(SwerveModuleState desired_state, boolean is_open_loop) {

        desired_state = Constants.optimize(desired_state, this.getState().angle);

        this.setAngle(desired_state);
        this.setSpeed(desired_state, is_open_loop);

    }

    public void setSpeed(SwerveModuleState desired_state, boolean is_open_loop) {

        if (is_open_loop) {

            double percent_output = desired_state.speedMetersPerSecond / SwerveConstants.max_speed;
            this.drive_motor.set(percent_output);

        } else {

            // check if FF needed
            this.drive_controller.setReference(desired_state.speedMetersPerSecond, ControlType.kVelocity, 0);

        }

    }

    public void setAngle(SwerveModuleState desired_state) {
   
        Rotation2d angle;
        if (Math.abs(desired_state.speedMetersPerSecond) <= SwerveConstants.max_speed * 0.01)
            angle = this.prev_angle;
        else
            angle = desired_state.angle;
        this.angle_controller.setReference(angle.getDegrees(), ControlType.kPosition);
        this.prev_angle = angle;

    }

    public Rotation2d getAngle() {

        return Rotation2d.fromDegrees(this.angle_encoder.getPosition());

    }

    public SwerveModuleState getState() {

        return new SwerveModuleState(this.drive_encoder.getVelocity(), this.getAngle());

    }

    public Rotation2d getCANCoder() {
        return Rotation2d.fromDegrees(CANCoder.getAbsolutePosition().getValue());
      }

}
