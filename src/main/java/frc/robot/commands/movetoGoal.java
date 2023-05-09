package frc.robot.commands;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class movetoGoal extends CommandBase {
  DoubleSupplier xCurrent;
  ProfiledPIDController xPPID;
  SimpleMotorFeedforward xFF;
  double xSetpoint;
  Drivetrain swerve;

  public movetoGoal(DoubleSupplier xCurrent, ProfiledPIDController xPPID, SimpleMotorFeedforward xFF,  double xSetpoint, Drivetrain swerve) {
    this.xCurrent = xCurrent;
    this.xPPID = xPPID;
    this.xFF = xFF;
    this.xSetpoint = xSetpoint;
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    xPPID.reset(xCurrent.getAsDouble());
  }

  @Override
  public void execute() {
    double xOutput = xPPID.calculate(xCurrent.getAsDouble(), xSetpoint) + xFF.calculate(xPPID.getSetpoint().velocity);
    swerve.joystickDrive(xOutput, 0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (xPPID.atGoal());
  }
}
