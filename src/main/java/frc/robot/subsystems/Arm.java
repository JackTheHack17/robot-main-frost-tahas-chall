package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Triangle;
import frc.robot.Constants.*;
import frc.lib.Telemetry;


public class Arm extends SubsystemBase {
    private final CANSparkMax mBiscep;
    private final CANSparkMax mBiscep2;
    private final CANSparkMax mElbow;
    private final CANSparkMax mElbow2;
    private final CANSparkMax mClaw;
    private final CANSparkMax mClaw2;
    private final RelativeEncoder biscepEncoder;  
    private final RelativeEncoder elbowEncoder;
    private final RelativeEncoder clawEncoder;
    private final SparkMaxPIDController biscepPID;  
    private final SparkMaxPIDController elbowPID;
    private final SparkMaxPIDController clawPID;
    private SparkMaxAlternateEncoder.Type kAltEncType;
/*    private final ArmFeedforward fBiscep;
    private final ArmFeedforward fElbow;
    private final ArmFeedforward fClaw;
    private final TrapezoidProfile.Constraints biscepProfile;
    private final TrapezoidProfile.Constraints elbowProfile;
    private final TrapezoidProfile.Constraints clawProfile; */

    public Arm() {    
        //Motors
        mBiscep = new CANSparkMax(1, MotorType.kBrushless);
        mBiscep2 = new CANSparkMax(1, MotorType.kBrushless);   
        mElbow = new CANSparkMax(2, MotorType.kBrushless);
        mElbow2 = new CANSparkMax(2, MotorType.kBrushless);
        mClaw = new CANSparkMax(3, MotorType.kBrushless);
        mClaw2 = new CANSparkMax(3, MotorType.kBrushless);

        mBiscep.setIdleMode(IdleMode.kBrake);
        mElbow.setIdleMode(IdleMode.kBrake);
        mClaw.setIdleMode(IdleMode.kBrake);
        mBiscep2.setIdleMode(IdleMode.kBrake);
        mElbow2.setIdleMode(IdleMode.kBrake);
        mClaw2.setIdleMode(IdleMode.kBrake);
        mBiscep2.follow(mBiscep);
        mClaw2.follow(mClaw);
        mElbow2.follow(mElbow);

        
        kAltEncType = SparkMaxAlternateEncoder.Type.kQuadrature;

        biscepEncoder = mBiscep.getAlternateEncoder(kAltEncType, 8192);
        elbowEncoder = mElbow.getAlternateEncoder(kAltEncType, 8192);
        clawEncoder = mClaw.getAlternateEncoder(kAltEncType, 8192);

        biscepEncoder.setPositionConversionFactor(360);
        elbowEncoder.setPositionConversionFactor(360);
        clawEncoder.setPositionConversionFactor(360);
/*
        biscepProfile = new TrapezoidProfile.Constraints(0, 0);
        elbowProfile = new TrapezoidProfile.Constraints(0, 0);
        clawProfile = new TrapezoidProfile.Constraints(0, 0);

        fBiscep = new ArmFeedforward(0, 0, 0, 0);
        fElbow = new ArmFeedforward(0, 0, 0, 0);
        fClaw = new ArmFeedforward(0, 0, 0, 0); */

        biscepPID = mBiscep.getPIDController(); 
        elbowPID = mElbow.getPIDController();
        clawPID = mClaw.getPIDController();

        configPID(0, 0, 0, 0, 0, 0, biscepEncoder, biscepPID);
        configPID(0, 0, 0, 0, 0, 0, elbowEncoder, elbowPID);
        configPID(0, 0, 0, 0, 0, 0, clawEncoder, clawPID);
    }

    public void movetopoint(double x, double y, double claw) {
        Triangle triangle = new Triangle(x, y, Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)));
        posArm(triangle.getAngleA() + (90 - Math.atan2(x, y)));
        posElbows(triangle.getAngleB());
        posClaws(claw);
    }

    public void setArm(double speed) {
        mBiscep.set(speed);
    }

    public void setElbows(double speed) {    
        mElbow.set(speed);  
    }

    public void setClaws(double speed) {    
        mClaw.set(speed);  
    }

    public void posArm(double angle) {
//        mBiscep.set(biscepPID.calculate(biscepEncoder.getPosition(), angle) + fBiscep.calculate(biscepEncoder.getPosition(), biscepEncoder.getVelocity()));
        biscepPID.setReference(angle, CANSparkMax.ControlType.kPosition);
    }

    public void posElbows(double angle) {
        biscepPID.setReference(angle, CANSparkMax.ControlType.kPosition);
    }

    public void posClaws(double angle) {
        biscepPID.setReference(angle, CANSparkMax.ControlType.kPosition);
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

    @Override  public void periodic() {
        Telemetry.setValue("POP/Biscep/speed", mBiscep.get());
        Telemetry.setValue("POP/Biscep/temp", mBiscep.getMotorTemperature());
        Telemetry.setValue("POP/Biscep/voltage", mBiscep.getAppliedOutput());
        Telemetry.setValue("POP/Biscep/statorcurrent", mBiscep.getOutputCurrent());
        Telemetry.setValue("POP/Biscep/position", biscepEncoder.getPosition());
        Telemetry.setValue("POP/BiscepFollower/speed2", mBiscep2.get());
        Telemetry.setValue("POP/BiscepFollower/temp2", mBiscep2.getMotorTemperature());
        Telemetry.setValue("POP/BiscepFollower/voltage2", mBiscep2.getAppliedOutput());
        Telemetry.setValue("POP/BiscepFollower/statorcurrent2", mBiscep2.getOutputCurrent());
        Telemetry.setValue("POP/Elbow/speed", mElbow.get());
        Telemetry.setValue("POP/Elbow/temp", mElbow.getMotorTemperature());
        Telemetry.setValue("POP/Elbow/voltage", mElbow.getAppliedOutput());
        Telemetry.setValue("POP/Elbow/statorcurrent", mElbow.getOutputCurrent());
        Telemetry.setValue("POP/Elbow/position", elbowEncoder.getPosition());
        Telemetry.setValue("POP/ElbowFollower/speed2", mElbow2.get());
        Telemetry.setValue("POP/ElbowFollower/temp2", mElbow2.getMotorTemperature());
        Telemetry.setValue("POP/ElbowFollower/voltage2", mElbow2.getAppliedOutput());
        Telemetry.setValue("POP/ElbowFollower/statorcurrent2", mElbow2.getOutputCurrent());
        Telemetry.setValue("POP/Claw/speed", mClaw.get());
        Telemetry.setValue("POP/Claw/temp", mClaw.getMotorTemperature());
        Telemetry.setValue("POP/Claw/voltage", mClaw.getAppliedOutput());
        Telemetry.setValue("POP/Claw/statorcurrent", mClaw.getOutputCurrent());
        Telemetry.setValue("POP/Claw/position", clawEncoder.getPosition());
        Telemetry.setValue("POP/ClawFollower/speed2", mClaw2.get());
        Telemetry.setValue("POP/ClawFollower/temp2", mClaw2.getMotorTemperature());
        Telemetry.setValue("POP/ClawFollower/voltage2", mClaw2.getAppliedOutput());
        Telemetry.setValue("POP/ClawFollower/statorcurrent2", mClaw2.getOutputCurrent());
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