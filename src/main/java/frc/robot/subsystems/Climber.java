
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

  private CANSparkMax leftClimber;
  private RelativeEncoder leftClimbEncoder;

  /** Creates a new ExampleSubsystem. */
  public Climber() {
    this.leftClimber = new CANSparkMax(ClimberConstants.left_climber_id, MotorType.kBrushless);
    leftClimber.setInverted(true);

    // Shuffleboard.getTab("Game").addDouble(
    //     "Climber" + " Pos", () -> leftClimbEncoder.getPosition()
    //   );
  }

  public void Run(double speed) {
    leftClimber.set(speed);
  }

  // public void Up(Trigger controlTrigger) {
  //   double getPos = leftClimbEncoder.getPosition();
  //   leftClimbEncoder.setPosition(5 + getPos);
  // }

  public void Down(Trigger controlTrigger) {

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