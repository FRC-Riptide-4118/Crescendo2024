package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

//Custom Imports
import frc.robot.subsystems.SwerveDrive;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SwerveConstants;

public class Zero extends Command {
  private SwerveDrive s_Swerve;

  public Zero(
      SwerveDrive s_Swerve) {
    this.s_Swerve = s_Swerve;
    addRequirements(this.s_Swerve);
  }

  @Override
  public void execute() {
    /* Get Values, Deadband*/

    /* Drive */
    s_Swerve.drive(
        new Translation2d(0, 0).times(SwerveConstants.maxSpeed),
        0, 
        false,
        false);
  }
}