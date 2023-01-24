// package frc.robot.commands;

// import frc.robot.Constants;
// import frc.robot.Robot;
// import frc.robot.subsystems.Drivetrain;
// import frc.robot.subsystems.Pigeon;

// import edu.wpi.first.wpilibj2.command.CommandBase;

// // This command self=balances on the charging station using gyroscope pitch as feedback
// public class BalanceCommand extends CommandBase {

//   private final Drivetrain m_DriveSubsystem;
//   private final Pigeon m_pigeon;

//   private double errorPitch;
//   private double errorRoll;
//   private double currentAngleRoll;
//   private double currentAnglePitch;
//   private double drivePowerRoll;
//   private double drivePowerPitch;

//   /** Command to use Gyro data to resist the tip angle from the beam - to stabalize and balanace */
//   public BalanceCommand(Drivetrain drivetrain, Pigeon pigeon) {
//     this.m_DriveSubsystem = drivetrain;
//     this.m_pigeon = pigeon;
//     addRequirements(m_DriveSubsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     // Uncomment the line below this to simulate the gyroscope axis with a controller joystick
//     // Double currentAngle = -1 * Robot.controller.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS) * 45;
//     this.currentAnglePitch = m_pigeon.getPitch();
//     this.currentAngleRoll = m_pigeon.getRoll(); 

//     errorPitch = Constants.BEAM_BALANCED_GOAL_DEGREES - currentAnglePitch;
//     errorRoll = Constants.BEAM_BALANCED_GOAL_DEGREES - currentAngleRoll;
//     drivePowerPitch = -Math.min(Constants.BEAM_BALANACED_DRIVE_KP * errorPitch, 1);
//     drivePowerRoll = -Math.min(Constants.BEAM_BALANACED_DRIVE_KP * errorRoll, 1);

//     // Our robot needed an extra push to drive up in reverse, probably due to weight imbalances
//     if (drivePowerRoll < 0) {
//       drivePowerRoll *= Constants.BACKWARDS_BALANCING_EXTRA_POWER_MULTIPLIER;
//     }
//     if (drivePowerPitch < 0) {
//       drivePowerPitch *= Constants.BACKWARDS_BALANCING_EXTRA_POWER_MULTIPLIER;
//     }

//     // Limit the max power
//     if (Math.abs(drivePowerPitch) > 0.4) {
//       drivePowerPitch = Math.copySign(0.4, drivePowerPitch);
//     }

//     if (Math.abs(drivePowerRoll) > 0.4) {
//       drivePowerRoll = Math.copySign(0.4, drivePowerRoll);
//     }

//     m_DriveSubsystem.joystickDrive(drivePowerPitch, currentAngleRoll, currentAnglePitch);
    
//     // Debugging Print Statments
//     System.out.println("Current Angle: " + currentAngleRoll);
//     System.out.println("Current Angle: " + currentAnglePitch);
//     System.out.println("Error Pitch " + errorPitch);
//     System.out.println("Drive Power: " + drivePowerPitch);
//     System.out.println("Error Roll " + errorRoll);
//     System.out.println("Drive Power Roll: " + drivePowerRoll);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     m_DriveSubsystem.stop();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return Math.abs(errorPitch) < Constants.BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES; // End the command when we are within the specified threshold of being 'flat' (gyroscope pitch of 0 degrees)
//   }
// }
