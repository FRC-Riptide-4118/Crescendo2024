
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.proto.Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  // Left Climber
  private CANSparkMax intake;
  private RelativeEncoder intakeEncoder;

  /** Creates a new ExampleSubsystem. */
  public Intake() {

    // Left Climber
    this.intake = new CANSparkMax(IntakeConstants.intake_id, MotorType.kBrushless);
    intake.setInverted(true);
    this.intakeEncoder = intake.getEncoder();
  }

  // Intake
  public void RunIntake(double speed) {
    intake.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}