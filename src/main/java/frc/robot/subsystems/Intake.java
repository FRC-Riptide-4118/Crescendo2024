
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.proto.Controller;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LauncherConstants;

public class Intake extends SubsystemBase {

  // Left Climber
  private CANSparkMax intake;
  private RelativeEncoder intakeEncoder;

  private PIDController intakePID;

  /** Creates a new ExampleSubsystem. */
  public Intake() {

    // Left Climber
    this.intake = new CANSparkMax(IntakeConstants.intake_id, MotorType.kBrushless);
    intake.setInverted(true);
    this.intakeEncoder = intake.getEncoder();
    this.intakePID = new PIDController(LauncherConstants.launcher_kp, LauncherConstants.launcher_ki, LauncherConstants.launcher_kd);

    Shuffleboard.getTab("Game").addDouble(
        "Intake" + "Pos", () -> intakeEncoder.getPosition()
    );

  }

  // Intake
  public void intake() {
    intake.set(0.5);
  }

  public void Outtake() {
    intake.set(-0.5);
  }

  public void Off() {
    intake.set(0);
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