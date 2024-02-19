package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

//Custom Imports
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SwerveConstants;

public class TeleOpClimber extends Command {
  private TeleOpClimber s_Climber;
  
  @Override
  public void execute() {
    /* Get Values, Deadband*/
    
  }
}