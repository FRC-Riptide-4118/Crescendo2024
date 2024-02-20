
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
import frc.robot.Constants.SlidesConstants;

public class Slides extends SubsystemBase {

  // Left Climber
  private CANSparkMax slides;
  private RelativeEncoder slidesEncoder;

  /** Creates a new ExampleSubsystem. */
  public Slides() {

    // Left Climber
    this.slides = new CANSparkMax(SlidesConstants.slides_id, MotorType.kBrushless);
    slides.setInverted(true);
    this.slidesEncoder = slides.getEncoder();

    Shuffleboard.getTab("Game").addDouble(
        "Slides" + " SlidesPos", () -> slidesEncoder.getPosition()
    );
  }

  // Left Climber
  public void SlidesRun(double speed) {
    slides.set(speed);
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