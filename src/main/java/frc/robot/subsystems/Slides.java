
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.proto.Controller;
import edu.wpi.first.math.util.Units;
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
  private PIDController slidesPID;

  /** Creates a new ExampleSubsystem. */
  public Slides() {

    // Slides Config
    this.slides = new CANSparkMax(SlidesConstants.slides_id, MotorType.kBrushless);
    slides.setInverted(false);
    this.slidesEncoder = slides.getEncoder();
    this.slidesPID = new PIDController(SlidesConstants.slides_kp, SlidesConstants.slides_ki, SlidesConstants.slides_kd);

    Shuffleboard.getTab("Game").addDouble(
        "Slides" + "Pos", () -> slidesEncoder.getPosition()
    );

    Shuffleboard.getTab("Game").addDouble(
        "Slides" + "InPos", () -> getPositionInches()
    );
  }

  // Slides
  public void SlidesUp() {
    slides.set(0.1);
  }

  public void SlidesDown() {
    slides.set(-0.1);
  }

  public void SlidesOff() {
    slides.set(0);
  }

  // PID Slides
  public void setSetpoint(double setpoint) {
    slidesPID.setSetpoint(setpoint);
  }

  public void runPID() {
    slides.set(slidesPID.calculate(getPositionInches()));
  }

  public double getPositionInches() {
    return (Units.rotationsToRadians(slidesEncoder.getPosition())/75)*0.8125;
  }

  // DON'T USE YET!!! MIGHT BREAK THINGS- PEOPLE WILL CRY!!! 
  public void incrementUp() {
    if(Math.abs(SlidesConstants.position1 - getPositionInches()) <= SlidesConstants.tolerance) {
      slidesPID.setSetpoint(SlidesConstants.position2);
    }
    else if(Math.abs(SlidesConstants.position2 - getPositionInches()) <= SlidesConstants.tolerance) {
      slidesPID.setSetpoint(SlidesConstants.position3);
    }
    else if(Math.abs(SlidesConstants.position3 - getPositionInches()) <= SlidesConstants.tolerance) {
      slidesPID.setSetpoint(SlidesConstants.position1);
    }
  }

  public void incrementDown() {
    if(Math.abs(SlidesConstants.position1 - getPositionInches()) <= SlidesConstants.tolerance) {
      slidesPID.setSetpoint(SlidesConstants.position3);
    }
    else if(Math.abs(SlidesConstants.position2 - getPositionInches()) <= SlidesConstants.tolerance) {
      slidesPID.setSetpoint(SlidesConstants.position1);
    }
    else if(Math.abs(SlidesConstants.position3 - getPositionInches()) <= SlidesConstants.tolerance) {
      slidesPID.setSetpoint(SlidesConstants.position2);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // runPID();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}