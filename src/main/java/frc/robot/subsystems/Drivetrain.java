package frc.robot.subsystems;
import static frc.robot.Constants.DRIVETRAIN.*;
import static frc.robot.Constants.CAN.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.SwerveModule;
import frc.lib.Telemetry;
import frc.robot.Constants.ARM.positions;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.HolonomicController;
import frc.robot.commands.moveToPosition;
import frc.robot.RobotContainer;

public class Drivetrain extends SubsystemBase {
  private Pigeon m_gyro;
  private Arm m_arm;
  private PinchersofPower m_claw;
  private Limelight m_limelight;

  private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    new Translation2d(  ROBOT_WIDTH_METERS / 2,  ROBOT_WIDTH_METERS / 2 ),
    new Translation2d(  ROBOT_WIDTH_METERS / 2, -ROBOT_WIDTH_METERS / 2 ),
    new Translation2d( -ROBOT_WIDTH_METERS / 2,  ROBOT_WIDTH_METERS / 2 ),
    new Translation2d( -ROBOT_WIDTH_METERS / 2, -ROBOT_WIDTH_METERS / 2 ) );

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

  private final PIDController FL_PID = new PIDController(0.0100, 0, 0.000); // 0.105
  private final PIDController FR_PID = new PIDController(0.0105, 0, 0.000);
  private final PIDController BL_PID = new PIDController(0.0105, 0, 0.000);
  private final PIDController BR_PID = new PIDController(0.0100, 0, 0.000);

  private final double FL_kF = AZIMUTH_kF;
  private final double FR_kF = AZIMUTH_kF;
  private final double BL_kF = AZIMUTH_kF;
  private final double BR_kF = 0.6;

  private SwerveModule[] swerveModules = new SwerveModule[4];

  private boolean isRobotOriented = false;
  
  private static final StatorCurrentLimitConfiguration DRIVE_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(true, 60, 60, 0);
  private static final StatorCurrentLimitConfiguration AZIMUTH_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(true, 20, 20, 0);

  private SwerveDrivePoseEstimator m_odometry;

  private List<Pose2d> _coneWaypoints = new ArrayList<Pose2d>();
  private List<Pose2d> _cubeWaypoints = new ArrayList<Pose2d>();

  private Pose2d _robotPose = new Pose2d();

  private double _translationKp = 2.40;// 2.35//1.8;//3.25;//2.75;//2.5;//2.1;//2;//0.018;//0.03;//0.004 0.001
  private double _translationKi = 0;
  private double _translationKd = 0;
  private double _rotationKp = 1.83;//1.83;// 2.5//12.5;//15;//0.00005
  private double _rotationKi = 0;
  private double _rotationKd = 0.085; // 0.1

  // private double _alignTranslationKp = SmartDashboard.getNumber("alignTranslateP", 2.35);//1.8;//3.25;//2.75;//2.5;//2.1;//2;//0.018;//0.03;//0.004 0.001
  // private double _alignTranslationKi = SmartDashboard.getNumber("alignTranslateI", 0);
  // private double _alignTranslationKd = SmartDashboard.getNumber("alignTranslateD", 0);
  // private double _alignRotationKp = SmartDashboard.getNumber("alignRotateP", 1.83);// 2.5//12.5;//15;//0.00005
  // private double _alignRotationKi = SmartDashboard.getNumber("alignRotateI", 0);
  // private double _alignRotationKd = SmartDashboard.getNumber("alignRotateD", 0.087); // 0.1

  private double _alignTranslationKp = 3.1; //5.5;
  private double _alignTranslationKi = 0;//0.;
  private double _alignTranslationKd = 0;
  private double _alignRotationKp = 5.8;//2.5;
  private double _alignRotationKi = 0;//.42;
  private double _alignRotationKd = 0;//.0;

  public Field2d field2d = new Field2d();

  private moveToPosition _moveToPosition;
  
  private Constraints _tranConstraints = new Constraints(4, 3);
  private Constraints _rotConstraints = new Constraints(120, 60);

  public Drivetrain(Pigeon m_gyro, Arm m_arm, PinchersofPower m_claw, Limelight m_limelight) {
    this.m_gyro = m_gyro;
    this.m_arm = m_arm;
    this.m_claw = m_claw;
    this.m_limelight = m_limelight;

    configPID(FL_PID);
    configPID(FR_PID);
    configPID(BL_PID);
    configPID(BR_PID);

    configDrive(FL_Drive);
    configDrive(FR_Drive);
    configDrive(BL_Drive);
    configDrive(BR_Drive);

    configPosition(FL_Position, FL_ECODER_OFFSET);
    configPosition(FR_Position, FR_ECODER_OFFSET);
    configPosition(BL_Position, BL_ECODER_OFFSET);
    configPosition(BR_Position, BR_ECODER_OFFSET);

    configAzimuth(FL_Azimuth, FL_Position, FL_PID.getP(), FL_PID.getD(), FL_kF);
    configAzimuth(FR_Azimuth, FR_Position, FR_PID.getP(), FR_PID.getD(), FR_kF);
    configAzimuth(BL_Azimuth, BL_Position, BL_PID.getP(), BL_PID.getD(), BL_kF);
    configAzimuth(BR_Azimuth, BR_Position, BR_PID.getP(), BR_PID.getD(), BR_kF);

    swerveModules[0] = new SwerveModule(FL_Drive, FL_Azimuth, FL_Position, FL_PID, FL_kF, "FL");
    swerveModules[1] = new SwerveModule(FR_Drive, FR_Azimuth, FR_Position, FR_PID, FR_kF, "FR");
    swerveModules[2] = new SwerveModule(BL_Drive, BL_Azimuth, BL_Position, BL_PID, BL_kF, "BL");
    swerveModules[3] = new SwerveModule(BR_Drive, BR_Azimuth, BR_Position, BR_PID, BR_kF, "BR");

    m_odometry = new SwerveDrivePoseEstimator(
      m_kinematics, 
      new Rotation2d(0), 
      getSwerveModulePositions(), 
      new Pose2d());

    Telemetry.setValue("drivetrain/PathPlanner/translationKp", _translationKp);
    Telemetry.setValue("drivetrain/PathPlanner/translationKi", _translationKi);
    Telemetry.setValue("drivetrain/PathPlanner/translationKd", _translationKd);
    Telemetry.setValue("drivetrain/PathPlanner/rotationKp", _rotationKp);
    Telemetry.setValue("drivetrain/PathPlanner/rotationKi", _rotationKi);
    Telemetry.setValue("drivetrain/PathPlanner/rotationKd", _rotationKd);

    shwerveDrive.restoreFactoryDefaults();
    shwerveDrive.clearFaults();
    shwerveDrive.setSmartCurrentLimit(60);
    shwerveDrive.setSecondaryCurrentLimit(60);
    shwerveDrive.burnFlash();

   if (RobotContainer.getDriverAlliance() == DriverStation.Alliance.Red) {
      _coneWaypoints.add(new Pose2d(0.76, 6.13, new Rotation2d(0)));
      _coneWaypoints.add(new Pose2d(0.76, 7.49, new Rotation2d(0)));
      _coneWaypoints.add(new Pose2d(14.75, 5.15 - 0.05, new Rotation2d(0)));
      _coneWaypoints.add(new Pose2d(14.75, 3.94 - 0.05, new Rotation2d(0)));
      _coneWaypoints.add(new Pose2d(14.75, 3.38 - 0.05, new Rotation2d(0)));
      _coneWaypoints.add(new Pose2d(14.75, 2.28 - 0.05, new Rotation2d(0)));
      _coneWaypoints.add(new Pose2d(14.75, 1.67, new Rotation2d(0)));
      _coneWaypoints.add(new Pose2d(14.75, 0.47 + 0.05, new Rotation2d(0)));
      _cubeWaypoints.add(new Pose2d(14.75, 1.13 - 0.05, new Rotation2d(0)));
      _cubeWaypoints.add(new Pose2d(14.75, 2.95 - 0.05, new Rotation2d(0)));
      _cubeWaypoints.add(new Pose2d(14.75, 4.52 - 0.05, new Rotation2d(0)));
      _cubeWaypoints.add(new Pose2d(0.76, 6.13, new Rotation2d(0)));
      _cubeWaypoints.add(new Pose2d(0.76, 7.49, new Rotation2d(0)));
    } else if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
      _coneWaypoints.add(new Pose2d(15.79, 7.33 + 0.02, new Rotation2d(0)));
      _coneWaypoints.add(new Pose2d(15.79, 6.00 + 0.02, new Rotation2d(0)));
      _coneWaypoints.add(new Pose2d(1.82, 5.05 + 0.02, new Rotation2d(0)));
      _coneWaypoints.add(new Pose2d(1.82, 3.84 + 0.02, new Rotation2d(0)));
      _coneWaypoints.add(new Pose2d(1.82, 3.28 + 0.02, new Rotation2d(0)));
      _coneWaypoints.add(new Pose2d(1.82, 2.18 + 0.02, new Rotation2d(0)));
      _coneWaypoints.add(new Pose2d(1.82, 1.60 + 0.02, new Rotation2d(0)));
      _coneWaypoints.add(new Pose2d(1.86, 0.47 + 0.02, new Rotation2d(0)));
      _cubeWaypoints.add(new Pose2d(1.86, 1.03 + 0.02, new Rotation2d(0)));
      _cubeWaypoints.add(new Pose2d(1.86, 2.75 + 0.02, new Rotation2d(0)));
      _cubeWaypoints.add(new Pose2d(1.86, 4.42 + 0.02, new Rotation2d(0)));
      _cubeWaypoints.add(new Pose2d(15.79, 7.33 + 0.02, new Rotation2d(0)));
      _cubeWaypoints.add(new Pose2d(15.79, 6.00 + 0.02, new Rotation2d(0)));
    }

    _moveToPosition = new moveToPosition(this);

    PathPlannerServer.startServer(6969);

    resetPose(m_limelight.getPose());
  }

  @Override
  public void periodic() {
    for(int i = 0; i <= 3; i++) swerveModules[i].telemetry();

    Telemetry.setValue("drivetrain/isRobotOriented", isRobotOriented);

    forwardKinematics = m_kinematics.toChassisSpeeds(
      swerveModules[0].getState(), 
      swerveModules[1].getState(), 
      swerveModules[2].getState(), 
      swerveModules[3].getState()
    );

    if ( m_limelight.hastarget()) m_odometry.addVisionMeasurement(
      m_limelight.getPose(), 
      Timer.getFPGATimestamp() - m_limelight.getLatency(),
      VecBuilder.fill(0.9, 0.9, 10000000));

    _robotPose = m_odometry.update(new Rotation2d(Math.toRadians(m_gyro.getYaw())), getSwerveModulePositions());

    poseToTelemetry(_robotPose, "/chassis/");
    Telemetry.setValue("drivetrain/chassis/robot/forwardSpeed", forwardKinematics.vxMetersPerSecond);
    Telemetry.setValue("drivetrain/chassis/robot/rightwardSpeed", -forwardKinematics.vyMetersPerSecond);
    Telemetry.setValue("drivetrain/chassis/clockwiseSpeed", Math.toDegrees(forwardKinematics.omegaRadiansPerSecond));
    Telemetry.setValue("drivetrain/chassis/field/DSawaySpeed", ( forwardKinematics.vxMetersPerSecond * Math.cos(Math.toRadians(m_gyro.getYaw())) - forwardKinematics.vyMetersPerSecond * Math.sin(Math.toRadians(m_gyro.getYaw()))));
    Telemetry.setValue("drivetrain/chassis/field/DSrightSpeed", ( -forwardKinematics.vyMetersPerSecond * Math.cos(Math.toRadians(m_gyro.getYaw())) - forwardKinematics.vxMetersPerSecond * Math.sin(Math.toRadians(m_gyro.getYaw()))));

    field2d.setRobotPose(_robotPose);
    SmartDashboard.putData(field2d);
  }

  public void joystickDrive(double LX, double LY, double RX) {
    if ( !isRobotOriented ) m_chassisSpeeds = 
      ChassisSpeeds.fromFieldRelativeSpeeds(
        LY * MAX_LINEAR_SPEED, 
        -LX * MAX_LINEAR_SPEED, 
        -RX * MAX_ROTATION_SPEED, 
        m_odometry.getEstimatedPosition().getRotation().plus(Rotation2d.fromDegrees(180)));

    else m_chassisSpeeds = new ChassisSpeeds(LY * MAX_LINEAR_SPEED, -LX * MAX_LINEAR_SPEED, -RX * MAX_ROTATION_SPEED);

    m_chassisSpeeds = discretize(m_chassisSpeeds);
    modules = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(modules, MAX_LINEAR_SPEED);
    for(int i = 0; i <= 3; i++) swerveModules[i].setDesiredState(modules[i]); 
  }

  // For autonomous
  public void driveFromModuleStates ( SwerveModuleState[] modules ) {
    modules = m_kinematics.toSwerveModuleStates( discretize( m_kinematics.toChassisSpeeds(modules) ) );
    SwerveDriveKinematics.desaturateWheelSpeeds(modules, MAX_LINEAR_SPEED);

    for(int i = 0; i <= 3; i++) swerveModules[i].setDesiredState(modules[i]); 
  }

  public void driveFromChassisSpeeds (ChassisSpeeds speeds) {
    modules = m_kinematics.toSwerveModuleStates( speeds );
    SwerveDriveKinematics.desaturateWheelSpeeds( modules, MAX_LINEAR_SPEED );

    for(int i = 0; i <= 3; i++) swerveModules[i].setDesiredState( modules[i] );
  }

  public void lockModules ( SwerveModuleState[] modules ) {
    SwerveDriveKinematics.desaturateWheelSpeeds(modules, MAX_LINEAR_SPEED);
    m_chassisSpeeds = discretize(m_kinematics.toChassisSpeeds(modules));
    
    modules = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);

    for(int i = 0; i <= 3; i++) swerveModules[i].setLockedState(modules[i]);
  }

  public void stopModules () { for(int i = 0; i <= 3; i++) swerveModules[i].stopMotors(); }

  public void shwerve ( double LX, double LY) {
    // 6in diameter wheels, 10:1 gearbox
    if (isRobotOriented) shwerveDrive.set(MathUtil.clamp(-LX*9, -1, 1));
    else noShwerve();
  }

  public void noShwerve () {
    shwerveDrive.set(0);
  }

  public Command moveToPositionCommand () {
    // resetPose(m_limelight.getPose());
    Pose2d actualPose = _robotPose; 

    Pose2d closest = actualPose.nearest(m_claw.wantCone() ? _coneWaypoints : _cubeWaypoints);
    if (closest == null) return new InstantCommand();

    poseToTelemetry(actualPose, "Align/startPose");
    poseToTelemetry(closest, "Align/choosenWaypoint");

    return pathToCommand( closest );
  }

  public Command pathToCommand (Pose2d target) {
    field2d.getObject("Goal").setPose(target);

    Pose2d edgePose = new Pose2d(
      m_odometry.getEstimatedPosition().getX(), 
      target.getY(), 
      target.getRotation() );

    Command toAlign = _moveToPosition.generateMoveToPositionCommand(
      edgePose,
      new Pose2d( 0.1, 0.1, Rotation2d.fromDegrees(5)),
      generateAlignmentController() );

    Command toGoal = _moveToPosition.generateMoveToPositionCommand( 
      target, 
      new Pose2d(), 
      generateAlignmentController() );

    SwerveModuleState[] startState = new SwerveModuleState[] {
      new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(90)) };

    Command initWheelPositions = 
      new InstantCommand(() -> lockModules( startState ), this)
      .repeatedly().withTimeout(0.75);

    return new SequentialCommandGroup(initWheelPositions, toAlign, toGoal);
  }

  public HolonomicController generateAlignmentController() {
    return new HolonomicController(
      new ProfiledPIDController(
        _alignTranslationKp, 
        _alignTranslationKi,
        _alignTranslationKd,
        _tranConstraints), 
      new ProfiledPIDController(
        _alignTranslationKp, 
        _alignTranslationKi,
        _alignTranslationKd,
        _tranConstraints), 
      new ProfiledPIDController(
        _alignRotationKp, 
        _alignRotationKi,
        _alignRotationKd,
        _rotConstraints));
  }

  public double getGyroAngle() { return m_gyro.getPitch(); }

  public void zeroGyro() { m_gyro.zeroYaw(); }

  public boolean toggleRobotOrient() { return isRobotOriented = !isRobotOriented; }

  public boolean getIsRobotOriented() { return isRobotOriented; }

  public void setRobotOriented(boolean _isRobotOriented) { isRobotOriented = _isRobotOriented; }

  public Pose2d getPose() { return _robotPose; }

  public ChassisSpeeds getChassisSpeeds() { return forwardKinematics; }

  public SwerveDriveKinematics getKinematics() { return m_kinematics; }

  public Pose2d getTargetPose() { return _moveToPosition.getTarget(); }

  private SwerveModulePosition[] getSwerveModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for(int i = 0; i <= 3; i++) positions[i] = swerveModules[i].getPosition();
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
        PathPlanner.getConstraintsFromPath(
          Telemetry.getValue("general/autonomous/selectedRoutine", "Mobility")));

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
      eventMap.put("coneMode", new InstantCommand( () -> { m_claw.setCone(true); m_claw.closeGrip(); m_claw.spinSlow(); } ));
      eventMap.put("cubeMode", new InstantCommand( () -> { m_claw.setCone(false); m_claw.openGrip(); } ));
      eventMap.put("wait", new WaitCommand(0.75));

      SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        () -> m_odometry.getEstimatedPosition(),
        this::resetPose,
        m_kinematics,
        new PIDConstants(_translationKp, _translationKi, _translationKd),
        new PIDConstants(_rotationKp, _rotationKi, _rotationKd),
        this::driveFromModuleStates,
        eventMap,
        true,
        this
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
        m_arm.moveToPositionTerminatingCommand(positions.Idle) );
    }
  }

  public void resetPose(Pose2d pose) {
    m_odometry.resetPosition(
      new Rotation2d( Math.toRadians( m_gyro.getYaw() ) ), 
      getSwerveModulePositions(), 
      pose);
  }

  private void configDrive (TalonFX motor) {
    configDrive(motor, DRIVE_kP, DRIVE_kF);
  }

  // public to avoid warnings
  public void configAzimuth (TalonFX motor, CANCoder position) {
    configAzimuth(motor, position, AZIMUTH_kP, AZIMUTH_kD, AZIMUTH_kF);
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

  private void configAzimuth (TalonFX motor, CANCoder position, double kP, double kD, double kF) {
    motor.configFactoryDefault();
    motor.setInverted(TalonFXInvertType.CounterClockwise);
    motor.setNeutralMode(NeutralMode.Brake);
    motor.configRemoteFeedbackFilter(position, 0);
    motor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
    motor.configSelectedFeedbackCoefficient(360 / (2048 * AZIMUTH_GEAR_RATIO));
    motor.configStatorCurrentLimit(AZIMUTH_CURRENT_LIMIT);
    motor.setSelectedSensorPosition(degreesToFalcon(position.getAbsolutePosition()));
    motor.config_kP(0, kP);
    motor.config_kD(0, kD);
    motor.config_kF(0, kF);
    motor.configNeutralDeadband(AZIMUTH_DEADBAND);
  }

  private void configPosition (CANCoder encoder, double offset) {
    encoder.configFactoryDefault();
    encoder.configMagnetOffset(offset);
    encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    encoder.setPositionToAbsolute();
  }

  private void configPID(PIDController controller) {
    controller.enableContinuousInput(0, 360);
    controller.setTolerance(0);
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

  public void poseToTelemetry(Pose2d pose, String key) {
    Telemetry.setValue("drivetrain/" + key + "/x", pose.getTranslation().getX());
    Telemetry.setValue("drivetrain/" + key + "/y", pose.getTranslation().getY());
    Telemetry.setValue("drivetrain/" + key + "/heading", pose.getRotation().getDegrees());
  }

  public static double degreesToFalcon(double degrees) {
    return degrees / (360.0 / ( AZIMUTH_GEAR_RATIO * 2048.0));
  }
}