package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Triangle;
import frc.robot.Constants.ARM;
import static frc.robot.Constants.ARM.JOINT_ANGLE_DEADZONE;
import frc.robot.Constants.CAN;
import frc.lib.Telemetry;

public class Arm extends SubsystemBase {
    private final CANSparkMax m_Biscep;
    private final CANSparkMax m_Biscep2;
    private final CANSparkMax m_Elbow;
    private final CANSparkMax m_Claw;
    private final RelativeEncoder m_biscepEncoder;  
    private final RelativeEncoder m_elbowEncoder;
    private final RelativeEncoder m_clawEncoder;
    private final SparkMaxPIDController m_biscepPID;  
    private final SparkMaxPIDController m_elbowPID;
    private final SparkMaxPIDController m_clawPID;
    private SparkMaxAlternateEncoder.Type kAltEncType;
    private PinchersofPower m_clawSubsystem;
    private double m_biscepTarget = 0;
    private double m_elbowTarget = 0;
    private double m_clawTarget = 0;

    public Arm(PinchersofPower m_claw) {
        m_clawSubsystem = m_claw;

        m_Biscep = new CANSparkMax(CAN.ARM_STAGE_1_ID, MotorType.kBrushless);
        m_Biscep2 = new CANSparkMax(CAN.ARM_STAGE_1_FOLLOWER_ID, MotorType.kBrushless);   
        m_Elbow = new CANSparkMax(CAN.ARM_STAGE_2_ID, MotorType.kBrushless);
        m_Claw = new CANSparkMax(CAN.ARM_STAGE_3_ID, MotorType.kBrushless);

        m_Biscep.setIdleMode(IdleMode.kBrake);
        m_Elbow.setIdleMode(IdleMode.kBrake);
        m_Claw.setIdleMode(IdleMode.kBrake);
        m_Biscep2.setIdleMode(IdleMode.kBrake);
        m_Biscep2.follow(m_Biscep);

        kAltEncType = SparkMaxAlternateEncoder.Type.kQuadrature;

        m_biscepEncoder = m_Biscep.getAlternateEncoder(kAltEncType, 8192);
        m_elbowEncoder = m_Elbow.getAlternateEncoder(kAltEncType, 8192);
        m_clawEncoder = m_Claw.getAlternateEncoder(kAltEncType, 8192);

        m_biscepEncoder.setPositionConversionFactor(360);
        m_elbowEncoder.setPositionConversionFactor(360);
        m_clawEncoder.setPositionConversionFactor(360);

        m_biscepEncoder.setVelocityConversionFactor(360);
        m_elbowEncoder.setVelocityConversionFactor(360);
        m_clawEncoder.setVelocityConversionFactor(360);

        m_biscepPID = m_Biscep.getPIDController(); 
        m_elbowPID = m_Elbow.getPIDController();
        m_clawPID = m_Claw.getPIDController();

        configPID(0, 0, 0, 0, 0, 0, m_biscepEncoder, m_biscepPID);
        configPID(0, 0, 0, 0, 0, 0, m_elbowEncoder, m_elbowPID);
        configPID(0, 0, 0, 0, 0, 0, m_clawEncoder, m_clawPID);
    }

    public void moveToPoint(double x, double y, double claw) {
        Triangle triangle = new Triangle(x, y, Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)));
        posArm(triangle.getAngleA() + (90 - Math.atan2(x, y)));
        posElbows(triangle.getAngleB());
        posClaws(claw);
    }

    public Boolean isAtTarget () {
        return (
            Math.abs(m_biscepEncoder.getPosition() - m_biscepTarget) < JOINT_ANGLE_DEADZONE &&
            Math.abs(m_elbowEncoder.getPosition() - m_elbowTarget) < JOINT_ANGLE_DEADZONE &&
            Math.abs(m_clawEncoder.getPosition() - m_clawTarget) < JOINT_ANGLE_DEADZONE
        );
    }

    public void setArm(double speed) {
        m_Biscep.set(speed);
    }

    public void setElbows(double speed) {    
        m_Elbow.set(speed);  
    }

    public void setClaws(double speed) {    
        m_Claw.set(speed);  
    }

    public void posArm(double angle) {
        m_biscepTarget = angle;
        m_biscepPID.setReference(angle, CANSparkMax.ControlType.kPosition);
    }

    public void posElbows(double angle) {
        m_elbowTarget = angle;
        m_elbowPID.setReference(angle, CANSparkMax.ControlType.kPosition);
    }

    public void posClaws(double angle) {
        m_clawTarget = angle;
        m_clawPID.setReference(angle, CANSparkMax.ControlType.kPosition);
    }

    public void lowArmScore() {
        posArm(ARM.LOW_ARM_ANG);
        posElbows(ARM.LOW_ELBOW_ANG);  
        posClaws(ARM.LOW_CLAW_ANG);
    }

    public void highArmScore() {
        posArm(ARM.HIGH_ARM_ANG);
        posElbows(ARM.HIGH_ELBOW_ANG);  
        posClaws(ARM.HIGH_CLAW_ANG);
    }
    
    public void idleArmScore() {
        posArm(ARM.IDLE_ARM_ANG);
        posElbows(ARM.IDLE_ELBOW_ANG);  
        posClaws(ARM.IDLE_CLAW_ANG);
    }

    public void fetch() {
        posArm(ARM.FETCH_ARM_ANG);
        posElbows(ARM.FETCH_ELBOW_ANG);  
        posClaws(ARM.FETCH_CLAW_ANG);
    }

    public Command midScoreCommand() {
        return new FunctionalCommand(
            () -> {}, 
            this::lowArmScore, 
            interrupted -> m_clawSubsystem.Notake(m_clawSubsystem),
            () -> this.isAtTarget() == true,
            this
        );
    }

    public Command highScoreCommand() {
        return new FunctionalCommand(
            () -> {}, 
            this::highArmScore, 
            interrupted -> m_clawSubsystem.Notake(m_clawSubsystem),
            () -> this.isAtTarget() == true,
            this
        );
    }
    
    public Command lowScoreCommand() {
        return new FunctionalCommand(
            () -> {}, 
            this::idleArmScore, 
            interrupted -> m_clawSubsystem.Notake(m_clawSubsystem),
            () -> this.isAtTarget() == true,
            this
        );
    }
    
    public Command fetchCommand() {
        return new FunctionalCommand(
            () -> {}, 
            this::fetch, 
            interrupted -> m_clawSubsystem.Notake(m_clawSubsystem),
            () -> this.isAtTarget() == true,
            this
        );
    }

    @Override  public void periodic() {
        Telemetry.setValue("POP/Biscep/speed", m_Biscep.get());
        Telemetry.setValue("POP/Biscep/temp", m_Biscep.getMotorTemperature());
        Telemetry.setValue("POP/Biscep/voltage", m_Biscep.getAppliedOutput());
        Telemetry.setValue("POP/Biscep/statorcurrent", m_Biscep.getOutputCurrent());
        Telemetry.setValue("POP/Biscep/position", m_biscepEncoder.getPosition());
        Telemetry.setValue("POP/Biscep/velocity", m_biscepEncoder.getVelocity());
        Telemetry.setValue("POP/BiscepFollower/speed2", m_Biscep2.get());
        Telemetry.setValue("POP/BiscepFollower/temp2", m_Biscep2.getMotorTemperature());
        Telemetry.setValue("POP/BiscepFollower/voltage2", m_Biscep2.getAppliedOutput());
        Telemetry.setValue("POP/BiscepFollower/statorcurrent2", m_Biscep2.getOutputCurrent());
        Telemetry.setValue("POP/Elbow/speed", m_Elbow.get());
        Telemetry.setValue("POP/Elbow/temp", m_Elbow.getMotorTemperature());
        Telemetry.setValue("POP/Elbow/voltage", m_Elbow.getAppliedOutput());
        Telemetry.setValue("POP/Elbow/statorcurrent", m_Elbow.getOutputCurrent());
        Telemetry.setValue("POP/Elbow/position", m_elbowEncoder.getPosition());
        Telemetry.setValue("POP/Elbow/velocity", m_elbowEncoder.getVelocity());
        Telemetry.setValue("POP/Claw/speed", m_Claw.get());
        Telemetry.setValue("POP/Claw/temp", m_Claw.getMotorTemperature());
        Telemetry.setValue("POP/Claw/voltage", m_Claw.getAppliedOutput());
        Telemetry.setValue("POP/Claw/statorcurrent", m_Claw.getOutputCurrent());
        Telemetry.setValue("POP/Claw/position", m_clawEncoder.getPosition());
        Telemetry.setValue("POP/Claw/velocity", m_clawEncoder.getVelocity());
    }
    
    @Override  public void simulationPeriodic() {}

    public void configPID(double kp, double kd, double FF, double maxV, double maxA, int profile, RelativeEncoder encoder, SparkMaxPIDController controller) {
        controller.setP(kp, profile);
        controller.setD(kd, profile);
        controller.setFF(FF, profile);
        controller.setSmartMotionMaxAccel(maxA, profile);
        controller.setSmartMotionMaxVelocity(maxV, profile);
        controller.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, profile);
        controller.setFeedbackDevice(encoder);
    }
}