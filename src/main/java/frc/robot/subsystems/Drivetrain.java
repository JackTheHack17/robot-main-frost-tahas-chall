package frc.robot.subsystems;
import static frc.robot.Constants.CAN.BL_AZIMUTH_ID;
import static frc.robot.Constants.CAN.BL_CANCODER_ID;
import static frc.robot.Constants.CAN.BL_DRIVE_ID;
import static frc.robot.Constants.CAN.BR_AZIMUTH_ID;
import static frc.robot.Constants.CAN.BR_CANCODER_ID;
import static frc.robot.Constants.CAN.BR_DRIVE_ID;
import static frc.robot.Constants.CAN.FL_AZIMUTH_ID;
import static frc.robot.Constants.CAN.FL_CANCODER_ID;
import static frc.robot.Constants.CAN.FL_DRIVE_ID;
import static frc.robot.Constants.CAN.FR_AZIMUTH_ID;
import static frc.robot.Constants.CAN.FR_CANCODER_ID;
import static frc.robot.Constants.CAN.FR_DRIVE_ID;
import static frc.robot.Constants.CAN.SHWERVE_DRIVE_ID;
import static frc.robot.Constants.DRIVETRAIN.AUTO_BALANCE_Kd;
import static frc.robot.Constants.DRIVETRAIN.AUTO_BALANCE_Kp;
import static frc.robot.Constants.DRIVETRAIN.AZIMUTH_kD;
import static frc.robot.Constants.DRIVETRAIN.AZIMUTH_kP;
import static frc.robot.Constants.DRIVETRAIN.AZIMUTH_kF;
import static frc.robot.Constants.DRIVETRAIN.BL_ECODER_OFFSET;
import static frc.robot.Constants.DRIVETRAIN.BR_ECODER_OFFSET;
import static frc.robot.Constants.DRIVETRAIN.DRIVE_GEAR_RATIO;
import static frc.robot.Constants.DRIVETRAIN.DRIVE_kF;
import static frc.robot.Constants.DRIVETRAIN.DRIVE_kP;
import static frc.robot.Constants.DRIVETRAIN.FL_ECODER_OFFSET;
import static frc.robot.Constants.DRIVETRAIN.FR_ECODER_OFFSET;
import static frc.robot.Constants.DRIVETRAIN.MAX_LINEAR_SPEED;
import static frc.robot.Constants.DRIVETRAIN.MAX_ROTATION_SPEED;
import static frc.robot.Constants.DRIVETRAIN.ROBOT_WIDTH;
import static frc.robot.Constants.DRIVETRAIN.WHEEL_DIAMETER;
import static frc.robot.Constants.DRIVETRAIN.AZIMUTH_DEADBAND;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.server.PathPlannerServer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.Telemetry;
import frc.robot.Constants;
import frc.robot.Constants.ARM.positions;
import frc.robot.commands.AutoBalance;
import frc.robot.RobotContainer;

public class Drivetrain extends SubsystemBase {
  private Pigeon m_gyro;
  private Arm m_arm;
  private PinchersofPower m_claw;
  private Limelight m_limelight;

  private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    new Translation2d(ROBOT_WIDTH/2, ROBOT_WIDTH/2),
    new Translation2d(ROBOT_WIDTH/2, -ROBOT_WIDTH/2),
    new Translation2d(-ROBOT_WIDTH/2, ROBOT_WIDTH/2),
    new Translation2d(-ROBOT_WIDTH/2, -ROBOT_WIDTH/2)
  );

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds();
  private ChassisSpeeds forwardKinematics = new ChassisSpeeds();

  private SwerveModuleState[] modules;

  private final CANCoder FL_Position = new CANCoder(FL_CANCODER_ID, "drivetrain");
  private final CANCoder FR_Position = new CANCoder(FR_CANCODER_ID, "drivetrain");
  private final CANCoder BL_Position = new CANCoder(BL_CANCODER_ID, "drivetrain");
  private final CANCoder BR_Position = new CANCoder(BR_CANCODER_ID, "drivetrain");

  private final TalonFX FL_Drive = new TalonFX(FL_DRIVE_ID, "drivetrain");
  private final TalonFX FR_Drive = new TalonFX(FR_DRIVE_ID, "drivetrain");
  private final TalonFX BL_Drive = new TalonFX(BL_DRIVE_ID, "drivetrain");
  private final TalonFX BR_Drive = new TalonFX(BR_DRIVE_ID, "drivetrain");

  private final CANSparkMax shwerveDrive = new CANSparkMax(SHWERVE_DRIVE_ID, MotorType.kBrushless);

  private final TalonFX FL_Azimuth = new TalonFX(FL_AZIMUTH_ID, "drivetrain");
  private final TalonFX FR_Azimuth = new TalonFX(FR_AZIMUTH_ID, "drivetrain");
  private final TalonFX BL_Azimuth = new TalonFX(BL_AZIMUTH_ID, "drivetrain");
  private final TalonFX BR_Azimuth = new TalonFX(BR_AZIMUTH_ID, "drivetrain");

  private double FL_Target = 0.0;
  private double FR_Target = 0.0;
  private double BL_Target = 0.0;
  private double BR_Target = 0.0;

  private double FL_Speed = 0.0;
  private double FR_Speed = 0.0;
  private double BL_Speed = 0.0;
  private double BR_Speed = 0.0;

  private double FL_Actual_Position = 0.0;
  private double FR_Actual_Position = 0.0;
  private double BL_Actual_Position = 0.0;
  private double BR_Actual_Position = 0.0;

  private double FL_Actual_Speed = 0.0;
  private double FR_Actual_Speed = 0.0;
  private double BL_Actual_Speed = 0.0;
  private double BR_Actual_Speed = 0.0;

  private PIDController FL_PID = new PIDController(0.0100, 0, 0.000270); // 0.105
  private PIDController FR_PID = new PIDController(0.0105, 0, 0.000265);
  private PIDController BL_PID = new PIDController(0.0105, 0, 0.000265);
  private PIDController BR_PID = new PIDController(0.0100, 0, 0.000263);

  private boolean isRobotOriented = false;
  
  private static final StatorCurrentLimitConfiguration DRIVE_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(true, 60, 60, 0);
  private static final StatorCurrentLimitConfiguration AZIMUTH_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(true, 20, 20, 0);
  private static final double SCALER = 0.02;

  private SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
    m_kinematics, 
    new Rotation2d(0), 
    getSwerveModulePositions(), 
    new Pose2d());

  private List<Pose2d> _coneWaypoints = new ArrayList<Pose2d>();
  private List<Pose2d> _cubeWaypoints = new ArrayList<Pose2d>();

  private Pose2d _robotPose = new Pose2d();

  private double _translationKp = 2.40;// 2.35//1.8;//3.25;//2.75;//2.5;//2.1;//2;//0.018;//0.03;//0.004 0.001
  private double _translationKi = 0;
  private double _translationKd = 0;
  private double _rotationKp = 1.73;//1.83;// 2.5//12.5;//15;//0.00005
  private double _rotationKi = 0;
  private double _rotationKd = 0.085; // 0.1

  private double _alignTranslationKp = SmartDashboard.getNumber("alignTranslateP", 2.35);//1.8;//3.25;//2.75;//2.5;//2.1;//2;//0.018;//0.03;//0.004 0.001
  private double _alignTranslationKi = SmartDashboard.getNumber("alignTranslateI", 0);
  private double _alignTranslationKd = SmartDashboard.getNumber("alignTranslateD", 0);
  private double _alignRotationKp = SmartDashboard.getNumber("alignRotateP", 1.83);// 2.5//12.5;//15;//0.00005
  private double _alignRotationKi = SmartDashboard.getNumber("alignRotateI", 0);
  private double _alignRotationKd = SmartDashboard.getNumber("alignRotateD", 0.087); // 0.1

  private Field2d field2d = new Field2d();

  public Drivetrain(Pigeon m_gyro, Arm m_arm, PinchersofPower m_claw, Limelight m_limelight) {
    this.m_gyro = m_gyro;
    this.m_arm = m_arm;
    this.m_claw = m_claw;
    this.m_limelight = m_limelight;

    FL_PID.enableContinuousInput(0, 360);
    FR_PID.enableContinuousInput(0, 360);
    BL_PID.enableContinuousInput(0, 360);
    BR_PID.enableContinuousInput(0, 360);

    FL_PID.setTolerance(0);
    FR_PID.setTolerance(0);
    BL_PID.setTolerance(0);
    BR_PID.setTolerance(0);

    Telemetry.setValue("drivetrain/PathPlanner/translationKp", _translationKp);
    Telemetry.setValue("drivetrain/PathPlanner/translationKi", _translationKi);
    Telemetry.setValue("drivetrain/PathPlanner/translationKd", _translationKd);
    Telemetry.setValue("drivetrain/PathPlanner/rotationKp", _rotationKp);
    Telemetry.setValue("drivetrain/PathPlanner/rotationKi", _rotationKi);
    Telemetry.setValue("drivetrain/PathPlanner/rotationKd", _rotationKd);

    configDrive(FL_Drive);
    configDrive(FR_Drive);
    configDrive(BL_Drive);
    configDrive(BR_Drive);

    configPosition(FL_Position, FL_ECODER_OFFSET);
    configPosition(FR_Position, FR_ECODER_OFFSET);
    configPosition(BL_Position, BL_ECODER_OFFSET);
    configPosition(BR_Position, BR_ECODER_OFFSET);

    configAzimuth(FL_Azimuth, FL_Position);
    configAzimuth(FR_Azimuth, FR_Position);
    configAzimuth(BL_Azimuth, BL_Position);
    configAzimuth(BR_Azimuth, BR_Position);

    shwerveDrive.restoreFactoryDefaults();
    shwerveDrive.clearFaults();
    shwerveDrive.setSmartCurrentLimit(60);
    shwerveDrive.setSecondaryCurrentLimit(60);
    shwerveDrive.burnFlash();

    // declare scoring positions
    if (RobotContainer.getDriverAlliance() == DriverStation.Alliance.Red) {  
      // red alliance waypoints
      _coneWaypoints.add(new Pose2d(0.76, 6.13, new Rotation2d(0)));
      _coneWaypoints.add(new Pose2d(0.76, 7.49, new Rotation2d(0)));
      _coneWaypoints.add(new Pose2d(14.71, 5.15, new Rotation2d(Math.PI)));
      _coneWaypoints.add(new Pose2d(14.71, 3.94, new Rotation2d(Math.PI)));
      _coneWaypoints.add(new Pose2d(14.71, 3.38, new Rotation2d(Math.PI)));
      _coneWaypoints.add(new Pose2d(14.71, 2.28, new Rotation2d(Math.PI)));
      _coneWaypoints.add(new Pose2d(14.71, 1.70, new Rotation2d(Math.PI)));
      _coneWaypoints.add(new Pose2d(14.71, 0.57, new Rotation2d(Math.PI)));
      _cubeWaypoints.add(new Pose2d(14.71, 1.13, new Rotation2d(Math.PI)));
      _cubeWaypoints.add(new Pose2d(14.71, 2.95, new Rotation2d(Math.PI)));
      _cubeWaypoints.add(new Pose2d(14.71, 4.52, new Rotation2d(Math.PI)));
      _cubeWaypoints.add(new Pose2d(0.76, 6.13, new Rotation2d(0)));
      _cubeWaypoints.add(new Pose2d(0.76, 7.49, new Rotation2d(0)));
    } else if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
      // blue alliance waypoints
      _coneWaypoints.add(new Pose2d(15.79, 7.33, new Rotation2d(0)));
      _coneWaypoints.add(new Pose2d(15.79, 6.00, new Rotation2d(0)));
      _coneWaypoints.add(new Pose2d(1.86, 5.05, new Rotation2d(Math.PI)));
      _coneWaypoints.add(new Pose2d(1.86, 3.84, new Rotation2d(Math.PI)));
      _coneWaypoints.add(new Pose2d(1.86, 3.28, new Rotation2d(Math.PI)));
      _coneWaypoints.add(new Pose2d(1.86, 2.18, new Rotation2d(Math.PI)));
      _coneWaypoints.add(new Pose2d(1.86, 1.60, new Rotation2d(Math.PI)));
      _coneWaypoints.add(new Pose2d(1.86, 0.47, new Rotation2d(Math.PI)));
      _cubeWaypoints.add(new Pose2d(1.86, 1.03, new Rotation2d(Math.PI)));
      _cubeWaypoints.add(new Pose2d(1.86, 2.75, new Rotation2d(Math.PI)));
      _cubeWaypoints.add(new Pose2d(1.86, 4.42, new Rotation2d(Math.PI)));
      _cubeWaypoints.add(new Pose2d(15.79, 7.33, new Rotation2d(0)));
      _cubeWaypoints.add(new Pose2d(15.79, 6.00, new Rotation2d(0)));
    }

    PathPlannerServer.startServer(6969);
  }

  @Override
  public void periodic() {
    _translationKp = Telemetry.getValue("drivetrain/PathPlanner/translationKp", 0);
    _translationKi = Telemetry.getValue("drivetrain/PathPlanner/translationKi", 0);
    _translationKd = Telemetry.getValue("drivetrain/PathPlanner/translationKd", 0);
    _rotationKp = Telemetry.getValue("drivetrain/PathPlanner/rotationKp", 0);
    _rotationKi = Telemetry.getValue("drivetrain/PathPlanner/rotationKi", 0);
    _rotationKd = Telemetry.getValue("drivetrain/PathPlanner/rotationKd", 0);

    FL_Actual_Position = FL_Position.getAbsolutePosition();
    FR_Actual_Position = FR_Position.getAbsolutePosition();
    BL_Actual_Position = BL_Position.getAbsolutePosition();
    BR_Actual_Position = BR_Position.getAbsolutePosition();

    FL_Actual_Speed = 2.0*(((FL_Drive.getSelectedSensorVelocity() / 2048) * 10) / DRIVE_GEAR_RATIO) * Math.PI * WHEEL_DIAMETER;
    FR_Actual_Speed = 2.0*(((FR_Drive.getSelectedSensorVelocity() / 2048) * 10) / DRIVE_GEAR_RATIO) * Math.PI * WHEEL_DIAMETER;
    BL_Actual_Speed = 2.0*(((BL_Drive.getSelectedSensorVelocity() / 2048) * 10) / DRIVE_GEAR_RATIO) * Math.PI * WHEEL_DIAMETER;
    BR_Actual_Speed = 2.0*(((BR_Drive.getSelectedSensorVelocity() / 2048) * 10) / DRIVE_GEAR_RATIO) * Math.PI * WHEEL_DIAMETER;

    Telemetry.setValue("drivetrain/modules/FL/azimuth/targetPosition", FL_Target % 360);
    Telemetry.setValue("drivetrain/modules/FR/azimuth/targetPosition", FR_Target % 360);
    Telemetry.setValue("drivetrain/modules/BL/azimuth/targetPosition", BL_Target % 360);
    Telemetry.setValue("drivetrain/modules/BR/azimuth/targetPosition", BR_Target % 360);
    Telemetry.setValue("drivetrain/modules/FL/drive/targetSpeed", FL_Speed);
    Telemetry.setValue("drivetrain/modules/FR/drive/targetSpeed", FR_Speed);
    Telemetry.setValue("drivetrain/modules/BL/drive/targetSpeed", BL_Speed);
    Telemetry.setValue("drivetrain/modules/BR/drive/targetSpeed", BR_Speed);
    Telemetry.setValue("drivetrain/modules/FL/azimuth/actualPosition", FL_Actual_Position);
    Telemetry.setValue("drivetrain/modules/FR/azimuth/actualPosition", FR_Actual_Position);
    Telemetry.setValue("drivetrain/modules/BL/azimuth/actualPosition", BL_Actual_Position);
    Telemetry.setValue("drivetrain/modules/BR/azimuth/actualPosition", BR_Actual_Position);
    Telemetry.setValue("drivetrain/modules/FL/drive/actualSpeed", FL_Actual_Speed);
    Telemetry.setValue("drivetrain/modules/FR/drive/actualSpeed", FR_Actual_Speed);
    Telemetry.setValue("drivetrain/modules/BL/drive/actualSpeed", BL_Actual_Speed);
    Telemetry.setValue("drivetrain/modules/BR/drive/actualSpeed", BR_Actual_Speed);
    Telemetry.setValue("drivetrain/modules/FL/azimuth/outputVoltage", FL_Azimuth.getMotorOutputVoltage());
    Telemetry.setValue("drivetrain/modules/FR/azimuth/outputVoltage", FR_Azimuth.getMotorOutputVoltage());
    Telemetry.setValue("drivetrain/modules/BL/azimuth/outputVoltage", BL_Azimuth.getMotorOutputVoltage());
    Telemetry.setValue("drivetrain/modules/BR/azimuth/outputVoltage", BR_Azimuth.getMotorOutputVoltage());
    Telemetry.setValue("drivetrain/modules/FL/drive/outputVoltage", FL_Drive.getMotorOutputVoltage());
    Telemetry.setValue("drivetrain/modules/FR/drive/outputVoltage", FR_Drive.getMotorOutputVoltage());
    Telemetry.setValue("drivetrain/modules/BL/drive/outputVoltage", BL_Drive.getMotorOutputVoltage());
    Telemetry.setValue("drivetrain/modules/BR/drive/outputVoltage", BR_Drive.getMotorOutputVoltage());
    Telemetry.setValue("drivetrain/modules/FL/azimuth/statorCurrent", FL_Azimuth.getStatorCurrent());
    Telemetry.setValue("drivetrain/modules/FR/azimuth/statorCurrent", FR_Azimuth.getStatorCurrent());
    Telemetry.setValue("drivetrain/modules/BL/azimuth/statorCurrent", BL_Azimuth.getStatorCurrent());
    Telemetry.setValue("drivetrain/modules/BR/azimuth/statorCurrent", BR_Azimuth.getStatorCurrent());
    Telemetry.setValue("drivetrain/modules/FL/drive/statorCurrent", FL_Drive.getStatorCurrent());
    Telemetry.setValue("drivetrain/modules/FR/drive/statorCurrent", FR_Drive.getStatorCurrent());
    Telemetry.setValue("drivetrain/modules/BL/drive/statorCurrent", BL_Drive.getStatorCurrent());
    Telemetry.setValue("drivetrain/modules/BR/drive/statorCurrent", BR_Drive.getStatorCurrent());
    Telemetry.setValue("drivetrain/isRobotOriented", isRobotOriented);

    forwardKinematics = m_kinematics.toChassisSpeeds(
      new SwerveModuleState(Math.abs(FL_Actual_Speed), Rotation2d.fromDegrees(FL_Actual_Position)), 
      new SwerveModuleState(Math.abs(FR_Actual_Speed), Rotation2d.fromDegrees(FR_Actual_Position)), 
      new SwerveModuleState(Math.abs(BL_Actual_Speed), Rotation2d.fromDegrees(BL_Actual_Position)), 
      new SwerveModuleState(Math.abs(BR_Actual_Speed), Rotation2d.fromDegrees(BR_Actual_Position))
    );

    Telemetry.setValue("drivetrain/kinematics/robot/forwardSpeed", forwardKinematics.vxMetersPerSecond);
    Telemetry.setValue("drivetrain/kinematics/robot/rightwardSpeed", -forwardKinematics.vyMetersPerSecond);
    Telemetry.setValue("drivetrain/kinematics/clockwiseSpeed", Math.toDegrees(forwardKinematics.omegaRadiansPerSecond));
    Telemetry.setValue("drivetrain/kinematics/field/DSawaySpeed", ( forwardKinematics.vxMetersPerSecond * Math.cos(Math.toRadians(m_gyro.getYaw())) - forwardKinematics.vyMetersPerSecond * Math.sin(Math.toRadians(m_gyro.getYaw()))));
    Telemetry.setValue("drivetrain/kinematics/field/DSrightSpeed", ( -forwardKinematics.vyMetersPerSecond * Math.cos(Math.toRadians(m_gyro.getYaw())) - forwardKinematics.vxMetersPerSecond * Math.sin(Math.toRadians(m_gyro.getYaw()))));

    // if ( m_limelight.hastarget()) m_odometry.addVisionMeasurement(
    //   m_limelight.getPose(), 
    //   Timer.getFPGATimestamp() - m_limelight.getLatency(),
    //   VecBuilder.fill(0.9, 0.9, 5000));
    _robotPose = m_odometry.update(new Rotation2d(Math.toRadians(m_gyro.getYaw())), getSwerveModulePositions());

    Telemetry.setValue("drivetrain/odometry/field/DSawayPosition", _robotPose.getX());
    Telemetry.setValue("drivetrain/odometry/field/DSrightPosition", _robotPose.getY());

    field2d.setRobotPose(_robotPose);
    SmartDashboard.putData(field2d);
  }

  public void joystickDrive(double LX, double LY, double RX) {
    m_chassisSpeeds = new ChassisSpeeds(LY * MAX_LINEAR_SPEED, -LX * MAX_LINEAR_SPEED, -RX * MAX_ROTATION_SPEED);
    if ( !isRobotOriented ) m_chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(LY * MAX_LINEAR_SPEED, -LX * MAX_LINEAR_SPEED, -RX * MAX_ROTATION_SPEED, m_odometry.getEstimatedPosition().getRotation());
    
    m_chassisSpeeds = discretize(m_chassisSpeeds);
    modules = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);

    driveFromModuleStates(modules);
  }

  public void driveFromModuleStates ( SwerveModuleState[] modules ) {
    SwerveDriveKinematics.desaturateWheelSpeeds(modules, MAX_LINEAR_SPEED);
    m_chassisSpeeds = discretize(m_kinematics.toChassisSpeeds(modules));
    
    modules = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);

    modules[0] = SwerveModuleState.optimize(modules[0], new Rotation2d(Math.toRadians(FL_Position.getAbsolutePosition())));
    modules[1] = SwerveModuleState.optimize(modules[1], new Rotation2d(Math.toRadians(FR_Position.getAbsolutePosition())));
    modules[2] = SwerveModuleState.optimize(modules[2], new Rotation2d(Math.toRadians(BL_Position.getAbsolutePosition())));
    modules[3] = SwerveModuleState.optimize(modules[3], new Rotation2d(Math.toRadians(BR_Position.getAbsolutePosition())));

    FL_Speed = modules[0].speedMetersPerSecond;
    FR_Speed = modules[1].speedMetersPerSecond;
    BL_Speed = modules[2].speedMetersPerSecond;
    BR_Speed = modules[3].speedMetersPerSecond;

    FL_Target = modules[0].angle.getDegrees();
    FR_Target = modules[1].angle.getDegrees();
    BL_Target = modules[2].angle.getDegrees();
    BR_Target = modules[3].angle.getDegrees();

    FL_Target = Math.abs(FL_Speed) <= (MAX_LINEAR_SPEED * 0.01) ? FL_Actual_Position : FL_Target;
    BL_Target = Math.abs(BL_Speed) <= (MAX_LINEAR_SPEED * 0.01) ? BL_Actual_Position : BL_Target;
    FR_Target = Math.abs(FR_Speed) <= (MAX_LINEAR_SPEED * 0.01) ? FR_Actual_Position : FR_Target;
    BR_Target = Math.abs(BR_Speed) <= (MAX_LINEAR_SPEED * 0.01) ? BR_Actual_Position : BR_Target; 

    FL_Azimuth.set(ControlMode.PercentOutput, FL_PID.calculate(FL_Position.getAbsolutePosition(), FL_Target) + AZIMUTH_kF * Math.signum(FL_PID.getPositionError()));
    FR_Azimuth.set(ControlMode.PercentOutput, FR_PID.calculate(FR_Position.getAbsolutePosition(), FR_Target) + AZIMUTH_kF * Math.signum(FR_PID.getPositionError()));
    BL_Azimuth.set(ControlMode.PercentOutput, BL_PID.calculate(BL_Position.getAbsolutePosition(), BL_Target) + 0.06 * Math.signum(BL_PID.getPositionError()));
    BR_Azimuth.set(ControlMode.PercentOutput, BR_PID.calculate(BR_Position.getAbsolutePosition(), BR_Target) + 0.05 * Math.signum(BR_PID.getPositionError()));

    FL_Drive.set(ControlMode.Velocity, (FL_Speed*DRIVE_GEAR_RATIO/(Math.PI * WHEEL_DIAMETER)*2048)/10);
    FR_Drive.set(ControlMode.Velocity, (FR_Speed*DRIVE_GEAR_RATIO/(Math.PI * WHEEL_DIAMETER)*2048)/10);
    BL_Drive.set(ControlMode.Velocity, (BL_Speed*DRIVE_GEAR_RATIO/(Math.PI * WHEEL_DIAMETER)*2048)/10);
    BR_Drive.set(ControlMode.Velocity, (BR_Speed*DRIVE_GEAR_RATIO/(Math.PI * WHEEL_DIAMETER)*2048)/10);
  }

  public void stopModules () {  
    FL_Drive.set(ControlMode.PercentOutput, 0);
    FR_Drive.set(ControlMode.PercentOutput, 0);
    BL_Drive.set(ControlMode.PercentOutput, 0);
    BR_Drive.set(ControlMode.PercentOutput, 0);

    FL_Azimuth.set(ControlMode.PercentOutput, 0);
    FR_Azimuth.set(ControlMode.PercentOutput, 0);
    BL_Azimuth.set(ControlMode.PercentOutput, 0);
    BR_Azimuth.set(ControlMode.PercentOutput, 0);
  }

  public void lockModule(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(modules, MAX_LINEAR_SPEED);

    modules[0] = SwerveModuleState.optimize(modules[0], new Rotation2d(Math.toRadians(FL_Position.getAbsolutePosition())));
    modules[1] = SwerveModuleState.optimize(modules[1], new Rotation2d(Math.toRadians(FR_Position.getAbsolutePosition())));
    modules[2] = SwerveModuleState.optimize(modules[2], new Rotation2d(Math.toRadians(BL_Position.getAbsolutePosition())));
    modules[3] = SwerveModuleState.optimize(modules[3], new Rotation2d(Math.toRadians(BR_Position.getAbsolutePosition())));

    FL_Speed = modules[0].speedMetersPerSecond;
    FR_Speed = modules[1].speedMetersPerSecond;
    BL_Speed = modules[2].speedMetersPerSecond;
    BR_Speed = modules[3].speedMetersPerSecond;

    FL_Target = modules[0].angle.getDegrees();
    FR_Target = modules[1].angle.getDegrees();
    BL_Target = modules[2].angle.getDegrees();
    BR_Target = modules[3].angle.getDegrees();

    FL_Azimuth.set(ControlMode.PercentOutput, FL_PID.calculate(FL_Position.getAbsolutePosition(), FL_Target) + AZIMUTH_kF * Math.signum(FL_PID.getPositionError()));
    FR_Azimuth.set(ControlMode.PercentOutput, FR_PID.calculate(FR_Position.getAbsolutePosition(), FR_Target) + AZIMUTH_kF * Math.signum(FR_PID.getPositionError()));
    BL_Azimuth.set(ControlMode.PercentOutput, BL_PID.calculate(BL_Position.getAbsolutePosition(), BL_Target) + 0.06 * Math.signum(BL_PID.getPositionError()));
    BR_Azimuth.set(ControlMode.PercentOutput, BR_PID.calculate(BR_Position.getAbsolutePosition(), BR_Target) + 0.05 * Math.signum(BR_PID.getPositionError()));

    FL_Drive.set(ControlMode.Velocity, (FL_Speed*DRIVE_GEAR_RATIO/(Math.PI * WHEEL_DIAMETER)*2048)/10);
    FR_Drive.set(ControlMode.Velocity, (FR_Speed*DRIVE_GEAR_RATIO/(Math.PI * WHEEL_DIAMETER)*2048)/10);
    BL_Drive.set(ControlMode.Velocity, (BL_Speed*DRIVE_GEAR_RATIO/(Math.PI * WHEEL_DIAMETER)*2048)/10);
    BR_Drive.set(ControlMode.Velocity, (BR_Speed*DRIVE_GEAR_RATIO/(Math.PI * WHEEL_DIAMETER)*2048)/10);
  }

  public Command PPmoveToPositionCommand () {
    Pose2d actualPose = _robotPose;

    Telemetry.setValue("drivetrain/PathPlanner/X", actualPose.getX());
    Telemetry.setValue("drivetrain/PathPlanner/Y", actualPose.getY());
    Telemetry.setValue("drivetrain/PathPlanner/Angle", actualPose.getRotation().getDegrees());    

    Pose2d closest = actualPose.nearest(m_claw.wantCone() ? _coneWaypoints : _cubeWaypoints);
    if (closest == null) return new InstantCommand();

    Telemetry.setValue("drivetrain/PathPlanner/ChoosenWaypointX", closest.getX());
    Telemetry.setValue("drivetrain/PathPlanner/ChoosenWaypointY", closest.getY());
    Telemetry.setValue("drivetrain/PathPlanner/ChoosenWaypointAngle", closest.getRotation().getDegrees()); 

    return PPpathToCommand( closest );
  }

  public Command PPpathToCommand (Pose2d target) {
    PathConstraints pathConstraints = new PathConstraints(2, 1);

    PathPlannerTrajectory _alignToTarget = PathPlanner.generatePath(
     pathConstraints,
      new PathPoint(new Translation2d(
        m_odometry.getEstimatedPosition().getX(), 
        m_odometry.getEstimatedPosition().getY()), 
        new Rotation2d(Math.toRadians(m_gyro.getYaw()))
        ),

      new PathPoint(
        new Translation2d(
          m_odometry.getEstimatedPosition().getX(), 
          target.getY()), 
          target.getRotation()
        ) 
    );

    PathPlannerTrajectory _toTarget = PathPlanner.generatePath(
      pathConstraints,
      new PathPoint(
        new Translation2d(
          m_odometry.getEstimatedPosition().getX(), 
          target.getY()), 
          target.getRotation()),
// In Order to fuse everything into one path just take this path point add it to the align to target path point
// and then only use the align command not a sequential command, using 2 paths for testing purposes
      new PathPoint(
        new Translation2d(
          target.getX(), 
          target.getY()), 
          target.getRotation()
        )
    );

    PIDController tPID = new PIDController(_alignTranslationKp, _alignTranslationKi, _alignTranslationKd);
    tPID.setTolerance(0);
    tPID.setIntegratorRange(-0, 0);

    PIDController rPID = new PIDController(_alignRotationKp, _alignRotationKi, _alignRotationKd);
    rPID.setTolerance(0);
    rPID.setIntegratorRange(-0, 0);

    Command align = new PPSwerveControllerCommand(
      _alignToTarget,
      () -> m_odometry.getEstimatedPosition(),
      this.m_kinematics, 
      tPID, 
      tPID, 
      rPID, 
      this::driveFromModuleStates,
      (Subsystem) this
    );

    Command toGoal = new PPSwerveControllerCommand(
      _toTarget,
      () -> m_odometry.getEstimatedPosition(),
      this.m_kinematics, 
      tPID, 
      tPID, 
      rPID, 
      this::driveFromModuleStates,
      (Subsystem) this
    );

    SwerveModuleState[] startState = new SwerveModuleState[] {
      new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(90))};

    Command initWheelPositions = 
      new InstantCommand(() -> driveFromModuleStates(startState), this)
      .repeatedly().withTimeout(0.75);

    return new SequentialCommandGroup(initWheelPositions, align, toGoal);
  }

  public double getGyroAngle(){
    return m_gyro.getPitch();
  }

  public Command autoBalanceCommand () {
    PIDController pitchPID = new PIDController(AUTO_BALANCE_Kp, 0, AUTO_BALANCE_Kd);
    PIDController rollPID = new PIDController(AUTO_BALANCE_Kp, 0, AUTO_BALANCE_Kd);

    return new FunctionalCommand(
      () -> {}, 
      () -> {
        joystickDrive(pitchPID.calculate(m_gyro.getPitch(), 0), rollPID.calculate(m_gyro.getRoll(), 0), 0);
      }, 
      (interrupted) -> {
        pitchPID.close();
        rollPID.close();
      }, 
      () -> {return pitchPID.atSetpoint() && rollPID.atSetpoint();}, 
      (Subsystem) this
    ).repeatedly();
  }

  public void zeroGyro() {
    m_gyro.zeroYaw();
  }

  public boolean toggleRobotOrient() {
    isRobotOriented = !isRobotOriented;
    return isRobotOriented;
  }

  public boolean getIsRobotOriented() {
    return isRobotOriented;
  }

  public void setRobotOriented(boolean _isRobotOriented) {
    isRobotOriented = _isRobotOriented;
  }

  private SwerveModulePosition[] getSwerveModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    positions[0] = new SwerveModulePosition(SCALER*(FL_Drive.getSelectedSensorPosition() / 2048) * Constants.DRIVETRAIN.DRIVE_GEAR_RATIO * Constants.DRIVETRAIN.WHEEL_PERIMETER, Rotation2d.fromDegrees(FL_Actual_Position));
    positions[1] = new SwerveModulePosition(SCALER*(FR_Drive.getSelectedSensorPosition() / 2048) * Constants.DRIVETRAIN.DRIVE_GEAR_RATIO * Constants.DRIVETRAIN.WHEEL_PERIMETER, Rotation2d.fromDegrees(FR_Actual_Position));
    positions[2] = new SwerveModulePosition(SCALER*(BL_Drive.getSelectedSensorPosition() / 2048) * Constants.DRIVETRAIN.DRIVE_GEAR_RATIO * Constants.DRIVETRAIN.WHEEL_PERIMETER, Rotation2d.fromDegrees(BL_Actual_Position));
    positions[3] = new SwerveModulePosition(SCALER*(BR_Drive.getSelectedSensorPosition() / 2048) * Constants.DRIVETRAIN.DRIVE_GEAR_RATIO * Constants.DRIVETRAIN.WHEEL_PERIMETER, Rotation2d.fromDegrees(BR_Actual_Position));
    return positions;
  }

  public Command getAutonomousCommand () {
    if (Telemetry.getValue("general/autonomous/selectedRoutine", "dontMove").equals("special")) {
      return new InstantCommand(()->setRobotOriented(true)).andThen(new RepeatCommand(new InstantCommand(()->joystickDrive(0, 0.5, 0))).withTimeout(1).andThen(new InstantCommand(()->stopModules())));
    }

    // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
    // for every path in the group
    try {
      List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
        Telemetry.getValue("general/autonomous/selectedRoutine", "dontMove"), 
        //new PathConstraints(0.5, .25)
        PathPlanner.getConstraintsFromPath(Telemetry.getValue("general/autonomous/selectedRoutine", "Mobility"))
      );

      HashMap<String, Command> eventMap = new HashMap<>();
      eventMap.put("marker1", new PrintCommand("Passed marker 1"));
      eventMap.put("placeHighCone", m_arm.goToScoreHigh().withTimeout(1.5));
      eventMap.put("placeMidCone", m_arm.goToScoreMid().withTimeout(1.5));
      eventMap.put("placeHighCube", m_arm.moveToPositionTerminatingCommand(positions.ScoreHighCube).withTimeout(1.5));
      eventMap.put("tuck", m_arm.moveToPositionTerminatingCommand(positions.Idle).withTimeout(0.5));
      eventMap.put("release", m_claw.outTakeCommand().andThen(new WaitCommand(.25)));
      eventMap.put("pickupLow", m_arm.moveToPositionCommand(positions.AutonFloor).withTimeout(0.1));
      eventMap.put("pickupLowAlt", m_arm.moveToPositionCommand(positions.FloorAlt).withTimeout(0.85));
      eventMap.put("intake",(m_claw.intakeCommand().repeatedly().withTimeout(0.5)));
      eventMap.put("autobalance", new AutoBalance(this));
      eventMap.put("realign", PPmoveToPositionCommand());
      eventMap.put("coneMode", new InstantCommand( () -> { m_claw.setCone(true); m_claw.closeGrip(); m_claw.spinSlow(); } ));
      eventMap.put("cubeMode", new InstantCommand( () -> { m_claw.setCone(false); m_claw.openGrip(); } ));
      eventMap.put("wait", new WaitCommand(0.75));

      SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        () -> m_odometry.getEstimatedPosition(),
        pose -> m_odometry.resetPosition(new Rotation2d(Math.toRadians(m_gyro.getYaw())), getSwerveModulePositions(), pose),
        this.m_kinematics,
        new PIDConstants(_translationKp, _translationKi, _translationKd),
        new PIDConstants(_rotationKp, _rotationKi, _rotationKd),
        this::driveFromModuleStates,
        eventMap,
        true,
        (Subsystem) this
      );

      return autoBuilder.fullAuto(pathGroup);
    } catch (Exception e) {
      // uh oh
      DriverStation.reportError("it crashed LOL " + e.getLocalizedMessage(), true);

      // score a preloaded cone if the auton crashes
      return new SequentialCommandGroup(
        new InstantCommand( () -> stopModules() ),
        new InstantCommand( () -> { m_claw.setCone(true); m_claw.closeGrip(); } ),
        m_arm.moveToPositionTerminatingCommand(positions.ScoreHighCone).withTimeout(2.75).andThen(m_arm.moveToPositionCommand(positions.DipHighCone).withTimeout(0.75)),
        m_claw.outTakeCommand().andThen(new WaitCommand(.25)),
        m_arm.moveToPositionTerminatingCommand(positions.Idle)
      );
    }
  }

  public void resetPose(Pose2d pose) {
    m_odometry.resetPosition(
      new Rotation2d( Math.toRadians( m_gyro.getYaw() ) ), 
      getSwerveModulePositions(), 
      pose);
  }

  public void shwerve ( double LX, double LY) {
    // 6in diameter wheels, 10:1 gearbox
    if (isRobotOriented) {
      shwerveDrive.set(MathUtil.clamp(-LX*9, -1, 1));
    } else {
      noShwerve();
    }
  }

  public void noShwerve () {
    shwerveDrive.set(0);
  }


  private void configDrive (TalonFX motor) {
    configDrive(motor, DRIVE_kP, DRIVE_kF);
  }

  private void configAzimuth (TalonFX motor, CANCoder position) {
    configAzimuth(motor, position, AZIMUTH_kP, AZIMUTH_kD);
  }

  private void configDrive (TalonFX motor, double kP, double kF) {
    motor.configFactoryDefault();
    motor.setInverted(TalonFXInvertType.CounterClockwise);
    motor.setNeutralMode(NeutralMode.Brake);
    motor.configStatorCurrentLimit(DRIVE_CURRENT_LIMIT);
    motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    motor.setSelectedSensorPosition(0);
    motor.config_kP(0, kP);
    motor.config_kF(0, kF);
    motor.configVoltageCompSaturation(12);
    motor.enableVoltageCompensation(true);
  }

  private void configAzimuth (TalonFX motor, CANCoder position, double kP, double kD) {
    motor.configFactoryDefault();
    motor.setInverted(TalonFXInvertType.CounterClockwise);
    motor.setNeutralMode(NeutralMode.Brake);
    motor.configRemoteFeedbackFilter(position, 0);
    motor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
    motor.configStatorCurrentLimit(AZIMUTH_CURRENT_LIMIT);
    motor.setSelectedSensorPosition(position.getAbsolutePosition());
    motor.config_kP(0, AZIMUTH_kP);
    motor.config_kD(0, AZIMUTH_kD);
    motor.configNeutralDeadband(AZIMUTH_DEADBAND);
  }

  private void configPosition (CANCoder encoder, double offset) {
    encoder.configFactoryDefault();
    encoder.configMagnetOffset(offset);
    encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    encoder.setPositionToAbsolute();
  }

  public ChassisSpeeds discretize(ChassisSpeeds speeds) {
    double dt = 0.02;
    var desiredDeltaPose = new Pose2d(
      speeds.vxMetersPerSecond * dt, 
      speeds.vyMetersPerSecond * dt, 
      new Rotation2d(speeds.omegaRadiansPerSecond * dt * 3)
    );
    var twist = new Pose2d().log(desiredDeltaPose);

    return new ChassisSpeeds((twist.dx / dt), (twist.dy / dt), (speeds.omegaRadiansPerSecond));
  }
}