package frc.robot;
import java.io.File;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.lib.ButtonBoard;
import frc.lib.Telemetry;

import frc.robot.Constants.ARM.positions;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.PinchersofPower;
import frc.robot.subsystems.PinchersofPower.GamePieces;

public class RobotContainer {
  // Im leaving these ports as magic constants because there's no case where they are not these values
  public static final CommandXboxController driverController = new CommandXboxController(0);
  public static final ButtonBoard copilotController = new ButtonBoard(1, 2);

  public LEDs m_LEDs = new LEDs();
  public PinchersofPower m_claw = new PinchersofPower();
  public Arm m_arm = new Arm(m_claw, copilotController);
  public Drivetrain m_swerve = new Drivetrain( m_arm, m_claw );

  public RobotContainer() {
    m_claw.setArmPos(() -> m_arm.target);

    File[] paths = new File(Filesystem.getDeployDirectory(), "pathplanner").listFiles();
    String pathsString = "";
    for (int i = 0; i < paths.length; i++) {
      if (paths[i].isDirectory()) continue;
      pathsString += paths[i].getName().substring(0, paths[i].getName().indexOf(".")) + ",";
    }
    Telemetry.setValue("general/autonomous/availableRoutines", pathsString);
    Telemetry.setValue("general/autonomous/selectedRoutine", "SET ME");

    configureButtonBindings();

    m_arm.setDefaultCommand(m_arm.defaultCommand());
    m_swerve.setDefaultCommand(new DriveCommand(m_swerve, driverController, copilotController));
  }

  public Arm getArm() {
    return m_arm;
  }

  private void configureButtonBindings() {
    driverController.a().onTrue(new InstantCommand(m_swerve::zeroGyro) );

    driverController.b().onTrue(new InstantCommand(m_swerve::toggleRobotOrient) );

    driverController.y().onTrue(new InstantCommand(m_swerve::resetPoseWithLL ) );

    driverController.x()
      .whileTrue(new ScheduleCommand( m_swerve.moveToPositionCommand() ) )
      .onFalse(  new InstantCommand(  () -> {}, m_swerve ) );

    copilotController.button(0)
      .whileTrue(m_arm.moveToPositionCommand(positions.Substation))
      .onFalse(  m_claw.intakeCommand().alongWith(m_arm.moveToPositionCommand(positions.Idle)));

    copilotController.button(1)
      .whileTrue(m_arm.moveToPositionCommand(positions.Floor))
      .onFalse(  m_claw.intakeCommand());

    copilotController.button(2)
      .onTrue(   new InstantCommand( () -> m_arm.goToScoreHigh().schedule()))
      .onFalse(  m_arm.defaultCommand())
      .onFalse(  m_claw.intakeCommand());

    copilotController.button(3)
      .whileTrue(m_arm.moveToPositionCommand(positions.FloorAlt))
      .onFalse(  m_claw.intakeCommand());

    copilotController.button(4)
      .whileTrue(new InstantCommand( () -> m_arm.goToScoreMid().schedule()))
      .onFalse(  m_claw.intakeCommand())
      .onFalse(  m_arm.defaultCommand());

    copilotController.button(5)
      .whileTrue(m_arm.moveToPositionCommand(positions.ScoreLow))
      .onFalse(  m_claw.intakeCommand());

    copilotController.button(6)
      .onTrue(
        new SequentialCommandGroup(
          (m_claw.outTakeCommand()), 
          new WaitCommand(0.25), 
          m_arm.moveToPositionCommand(positions.Idle) ) )
      .onFalse(m_claw.spinOffCommand());

    copilotController.button(7).onTrue(setGamePiece( GamePieces.Cube ));
    copilotController.button(8).onTrue(setGamePiece( GamePieces.Cone ));

    copilotController.button(9)
      .onTrue( m_arm.defaultCommand().alongWith(m_arm.onManual()))
      .onFalse(m_arm.defaultCommand());
    
    copilotController.button(12)
      .onTrue(    new InstantCommand( () -> { if ( isManual() ) m_claw.toggle(); } ) );

    copilotController.button(14)
      .whileTrue( new InstantCommand( () -> { if ( isManual() ) m_claw.spinOut(); } ) )
      .onFalse(   new InstantCommand( () -> { if ( isManual() ) m_claw.spinOff(); } ) );

    copilotController.button(13)
      .whileTrue( new InstantCommand( () -> { if ( isManual() ) m_claw.spinIn(); } ) )
      .onFalse(   new InstantCommand( () -> { if ( isManual() ) m_claw.spinOff(); } ) );

    copilotController.button(10)
      .whileTrue( new InstantCommand( () -> { if ( isManual() ) { m_arm.pushTargetTheta(); } } ).repeatedly() );

    copilotController.button(11)
      .whileTrue( new InstantCommand( () -> { if ( isManual() ) { m_arm.pushTargetTheta(); } } ).repeatedly() );
  }

  public void killRumble(){
    driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
  }

  public Command getAutonomousCommand() {
    return m_swerve.getAutonomousCommand().andThen(new InstantCommand( () -> m_swerve.stopModules()));
  }

  public static DriverStation.Alliance getDriverAlliance() {
    return DriverStation.getAlliance();
  }

  public static boolean isManual() {
    return copilotController.getRawButton(9);
  }

  public Command setGamePiece(GamePieces GP) {
    boolean isCone = GP.equals(GamePieces.Cone);
    Command updateMode = new InstantCommand( () -> {
      m_claw.setMode(GP);
      m_claw.setCone(isCone);
      copilotController.setLED(7, !isCone);
      copilotController.setLED(8, isCone);
    });

    if(isCone) return m_LEDs.turnYellow().alongWith(updateMode);
    else return m_LEDs.turnPurple().alongWith(updateMode);
  }

  public static void setCopilotLEDs() {
    if (!DriverStation.isAutonomous()) {
      setManualLEDs(isManual());
      setAutoLEDs(!isManual());
    }
    else {
      setManualLEDs(false);
      setAutoLEDs(false);
    }
  }

  public static void setManualLEDs(boolean isOn) {
    for(int i = 0; i <= 6; i++ ) copilotController.setLED(i, isOn);
  }

  public static void setAutoLEDs(boolean isOn) {
    for(int i = 10; i <= 14; i++) copilotController.setLED(i, isOn);
  }
}
