
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.proto.Controller;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ControllerConstants;

public class Climber extends SubsystemBase {

  // Left Climber
  private CANSparkMax leftClimber;
  private RelativeEncoder leftClimbEncoder;

  // Right Climber
  private CANSparkMax rightClimber;
  private RelativeEncoder rightClimbEncoder;

  /** Creates a new ExampleSubsystem. */
  public Climber() {

    // Left Climber
    this.leftClimber = new CANSparkMax(ClimberConstants.left_climber_id, MotorType.kBrushless);
    leftClimber.setInverted(true);
    this.leftClimbEncoder = leftClimber.getEncoder();
    leftClimber.setSmartCurrentLimit(ClimberConstants.left_climber_current_limit);
    leftClimber.setSecondaryCurrentLimit(ClimberConstants.left_climber_current_limit);

    Shuffleboard.getTab("Game").addDouble(
        "Climber" + " LeftPos", () -> leftClimbEncoder.getPosition()
    );

    // Right Climber
    this.rightClimber = new CANSparkMax(ClimberConstants.right_climber_id, MotorType.kBrushless);
    rightClimber.setInverted(false);
    this.rightClimbEncoder = rightClimber.getEncoder();
    rightClimber.setSmartCurrentLimit(ClimberConstants.right_climber_current_limit);
    rightClimber.setSecondaryCurrentLimit(ClimberConstants.right_climber_current_limit);

    Shuffleboard.getTab("Game").addDouble(
        "Climber" + " RightPos", () -> rightClimbEncoder.getPosition()
    );
    
  }

  // Left Climber
  public void LeftRun(double speed) {
    leftClimber.set(speed);
  }

  // Right Climber
  public void RightRun(double speed) {
    rightClimber.set(speed);
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