
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {

  private CANSparkMax leftClimber;

  /** Creates a new ExampleSubsystem. */
  public Climber() {
    this.leftClimber = new CANSparkMax(ClimberConstants.left_climber_id, MotorType.kBrushless);
    leftClimber.setInverted(true);
  }

  public void Run(double speed) {
    leftClimber.set(speed);
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