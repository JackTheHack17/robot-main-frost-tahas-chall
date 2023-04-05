// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import static frc.robot.Constants.DRIVETRAIN.ROTATION_SCALE_FACTOR;
import static frc.robot.Constants.DRIVETRAIN.SWERVE_SLOW_SPEED_PERCENTAGE;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.lib.ButtonBoard;
import frc.robot.subsystems.Drivetrain;

/** An example command that uses an example subsystem. */
public class DriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain m_swerve;
  
  // controller axis values
  private double m_LX = 0.0;
  private double m_LY = 0.0;
  private double m_RX = 0.0;

  private CommandGenericHID driverController;
  //private ButtonBoard copilotController; // commented to appease linter

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCommand(Drivetrain drivetrain, CommandGenericHID driverController, ButtonBoard copilotController) {
    m_swerve = drivetrain;
    this.driverController = driverController;
    //this.copilotController = copilotController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (DriverStation.isAutonomous()) return;

    // fetch joystick axis values
    m_LX = driverController.getRawAxis(0); // left x axis (strafe)
    m_LY = -driverController.getRawAxis(1); // left y axis (strafe)
    m_RX = driverController.getRawAxis(4); // right x axis (rotation)

    // deadzones
    m_LX = ( Math.abs(m_LX) < 0.1 ) ? 0 : m_LX;
    m_LY = ( Math.abs(m_LY) < 0.1 ) ? 0 : m_LY;
    m_RX = ( Math.abs(m_RX) < 0.1 ) ? 0 : m_RX;

    // square joysticks
    m_LX = m_LX * m_LX * ( Math.abs(m_LX) / (m_LX == 0 ? 1 : m_LX ) );
    m_LY = m_LY * m_LY * ( Math.abs(m_LY) / (m_LY == 0 ? 1 : m_LY ) );
    m_RX = m_RX * m_RX * ( Math.abs(m_RX) / (m_RX == 0 ? 1 : m_RX ) );

    m_RX *= ROTATION_SCALE_FACTOR;

    if ( driverController.getHID().getRawButton(6) ) m_RX += SWERVE_SLOW_SPEED_PERCENTAGE;
    if ( driverController.getHID().getRawButton(5) ) m_RX -= SWERVE_SLOW_SPEED_PERCENTAGE;

    if ( driverController.getHID().getPOV() == -1 ) {
      m_swerve.joystickDrive(m_LX, m_LY, m_RX);
    } else {
      m_swerve.driveFromModuleStates(new SwerveModuleState[]{
        new SwerveModuleState((m_swerve.modulesInPosition() ? SWERVE_SLOW_SPEED_PERCENTAGE : 0), Rotation2d.fromDegrees(driverController.getHID().getPOV())),
        new SwerveModuleState((m_swerve.modulesInPosition() ? SWERVE_SLOW_SPEED_PERCENTAGE : 0), Rotation2d.fromDegrees(driverController.getHID().getPOV())),
        new SwerveModuleState((m_swerve.modulesInPosition() ? SWERVE_SLOW_SPEED_PERCENTAGE : 0), Rotation2d.fromDegrees(driverController.getHID().getPOV())),
        new SwerveModuleState((m_swerve.modulesInPosition() ? SWERVE_SLOW_SPEED_PERCENTAGE : 0), Rotation2d.fromDegrees(driverController.getHID().getPOV()))
      });
    }
    
    // shwerve ded
    // if ( copilotController.getRawButton(16) ) {
    //   // shwerve engaged
    //   copilotController.setLED(16, true);
    //   m_swerve.shwerve(m_LX, m_LY);
    // } else {
    //   m_swerve.noShwerve();
    //   copilotController.setLED(16, false);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.joystickDrive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}