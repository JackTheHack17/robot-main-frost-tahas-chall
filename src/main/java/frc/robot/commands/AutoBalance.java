package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBalance extends ProfiledPIDCommand {
  /** Creates a new AutoBalance. */
  public AutoBalance(Drivetrain drivetrain ) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            0.006,
            0,
            0.0005,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(1, 1)),
        // This should return the measurement
        drivetrain::getGyroAngle, //gyro angle
        // This should return the goal (can also be a constant)
        0,
        // This uses the output
        (output, setpoint) -> {
          // Use the output (and setpoint, if desired) here
          drivetrain.joystickDrive(-output/2, 0, 0);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(drivetrain);
    getController().setTolerance(5); //degrees
  }

  public void initialize () {}

  @Override
  public boolean isFinished () {
    return false;
  }

  public void end (boolean interrupted) {}
}