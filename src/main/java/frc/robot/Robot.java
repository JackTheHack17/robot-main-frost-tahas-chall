// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.Telemetry;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private PowerDistribution PDH = new PowerDistribution();

  //private UsbCamera rawCamera;
  //private CvSource outputCamera;
  //private CvSink inputCamera;
  //private Mat mat;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.

    //rawCamera = CameraServer.startAutomaticCapture("raw camera", 0);
    //rawCamera.setFPS(30);
    //rawCamera.setResolution(640, 480);

    //outputCamera = CameraServer.putVideo("Aim Camera", 640, 480);
    //inputCamera = CameraServer.getVideo();

    //mat = new Mat();

    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    //inputCamera.grabFrame(mat);

    //Imgproc.line(mat, new Point(320,480), new Point(320,0), new Scalar(255, 0, 0));

    //outputCamera.putFrame(mat);

    Telemetry.setValue("general/PDH/voltage", PDH.getVoltage());
    Telemetry.setValue("general/PDH/current", PDH.getTotalCurrent());
    Telemetry.setValue("general/PDH/temperature", PDH.getTemperature());

    Telemetry.setValue("buttonBoard/joystick", "" + RobotContainer.copilotController.getJoystick().getX() + ", " + RobotContainer.copilotController.getJoystick().getY());

    Telemetry.setValue("general/FMSAlliance", DriverStation.getAlliance() == Alliance.Blue ? "Blue" : (DriverStation.getAlliance() == Alliance.Red ? "Red" : "Invalid") );
    
    if (!DriverStation.isEnabled()) {
      if ( RobotContainer.copilotController.getRawButton(9) ) {
        //DriverStation.reportWarning("Manual Override Enabled! Do not Enable Robot without Disabling Manual Override!", false);
      }
    }

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_robotContainer.killRumble();

    // schedule the autonomous command (example)
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
