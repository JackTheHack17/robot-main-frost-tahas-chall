package frc.robot.commands;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Align extends CommandBase {
  DoubleSupplier yCurrent;
  DoubleSupplier rotationCurrent;
  TrapezoidProfile.Constraints yProfile;
  TrapezoidProfile.Constraints rotProfile;
  ProfiledPIDController yPPID;
  ProfiledPIDController rotPPID;
  double ySetpoint;
  double rotationSetpoint;
  Drivetrain swerve;

  public Align(DoubleSupplier yCurrent, DoubleSupplier rotationCurrent, ProfiledPIDController yPPID, ProfiledPIDController rotPPID,  double ySetpoint, double rotationSetpoint, Drivetrain swerve) {
    this.yCurrent = yCurrent;
    this.rotationCurrent = rotationCurrent;
    this.yPPID = yPPID;
    this.rotPPID = rotPPID;
    this.ySetpoint = ySetpoint;
    this.rotationSetpoint = rotationSetpoint;
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    yPPID.reset(yCurrent.getAsDouble());
    rotPPID.reset(rotationCurrent.getAsDouble());
  }

  @Override
  public void execute() {
    double yOutput = yPPID.calculate(yCurrent.getAsDouble(), ySetpoint);
    double rotOutput = rotPPID.calculate(rotationCurrent.getAsDouble(), rotationSetpoint);
    swerve.joystickDrive(0, yOutput, rotOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (yPPID.atGoal() && rotPPID.atGoal());
  }
}
