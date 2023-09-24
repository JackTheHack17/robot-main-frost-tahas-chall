package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.Telemetry;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private PowerDistribution PDH = new PowerDistribution();

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    Telemetry.setValue("general/PDH/voltage", PDH.getVoltage());
    Telemetry.setValue("general/PDH/current", PDH.getTotalCurrent());
    Telemetry.setValue("general/PDH/temperature", PDH.getTemperature());

    Telemetry.setValue("buttonBoard/joystick", "" + RobotContainer.copilotController.getJoystick().getX() + ", " + RobotContainer.copilotController.getJoystick().getY());

    Telemetry.setValue("general/FMSAlliance", DriverStation.getAlliance() == Alliance.Blue ? "Blue" : (DriverStation.getAlliance() == Alliance.Red ? "Red" : "Invalid") );
    
    if (!DriverStation.isEnabled()) 
      if ( RobotContainer.copilotController.getRawButton(9) ) {}

    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_robotContainer.killRumble();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    m_robotContainer.killRumble();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.m_swerve.setRobotOriented(false);
  }
}
