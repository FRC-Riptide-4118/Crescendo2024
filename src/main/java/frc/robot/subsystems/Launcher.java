
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.proto.Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LauncherConstants;

public class Launcher extends SubsystemBase {

  // Left Climber
  private CANSparkMax launcher;
  private RelativeEncoder launcherEncoder;

  /** Creates a new ExampleSubsystem. */
  public Launcher() {

    // Left Climber
    this.launcher = new CANSparkMax(LauncherConstants.launcher_id, MotorType.kBrushless);
    launcher.setInverted(true);
    this.launcherEncoder = launcher.getEncoder();
  }

  // Intake
  public void RunLauncher(double speed) {
    launcher.set(speed);
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