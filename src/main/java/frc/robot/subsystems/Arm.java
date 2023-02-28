package frc.robot.subsystems;

import static frc.robot.Constants.ARM.*;

import java.util.HashMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.ArmPosition;
import frc.lib.ButtonBoard;
import frc.lib.Telemetry;
import frc.lib.Triangle;
import frc.robot.Constants.ARM.positions;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DIO;
import frc.robot.RobotContainer;

public class Arm extends SubsystemBase {
    private final CANSparkMax m_stage1;
    private final CANSparkMax m_stage2;
    private final CANSparkMax m_stage3;
    private final DutyCycleEncoder m_stage1Encoder;  
    private final DutyCycleEncoder m_stage2Encoder;
    private final DutyCycleEncoder m_stage3Encoder;
    private final SparkMaxPIDController m_stage1PID;  
    private final SparkMaxPIDController m_stage2PID;
    private final SparkMaxPIDController m_stage3PID;
    private PinchersofPower m_clawSubsystem;
    private LEDs m_LEDsSubsystem;
    private CommandXboxController m_driverController;
    private ButtonBoard m_copilotController;
    private double m_stage1Target = 0;
    private double m_stage2Target = 0;
    private double m_stage3Target = 0;
    private double m_manualTargetX = 0;
    private double m_manualTargetY = 0;
    private double m_manualTargetTheta = 0;
    private HashMap<positions, ArmPosition> positionMap = new HashMap<positions, ArmPosition>();

    public Arm(PinchersofPower m_claw, LEDs m_LEDs, CommandXboxController driverController, ButtonBoard copilotController) {
        // populate position map
        positionMap.put(positions.ScoreHigh, scoreHighPosition);
        positionMap.put(positions.ScoreMid, scoreMidPosition);
        positionMap.put(positions.ScoreLow, scoreLowPosition);
        positionMap.put(positions.Floor, floorPosition);
        positionMap.put(positions.FloorAlt, floorAltPosition);
        positionMap.put(positions.Substation, substationPosition);
        positionMap.put(positions.Idle, idlePosition);

        m_LEDsSubsystem = m_LEDs;
        m_clawSubsystem = m_claw;
        m_driverController = driverController;
        m_copilotController = copilotController;

        m_stage1 = new CANSparkMax(CAN.ARM_STAGE_1_ID, MotorType.kBrushless);
        m_stage2 = new CANSparkMax(CAN.ARM_STAGE_2_ID, MotorType.kBrushless);
        m_stage3 = new CANSparkMax(CAN.ARM_STAGE_3_ID, MotorType.kBrushless);

        m_stage1.setIdleMode(IdleMode.kBrake);
        m_stage2.setIdleMode(IdleMode.kBrake);
        m_stage3.setIdleMode(IdleMode.kBrake);

        m_stage1Encoder = new DutyCycleEncoder(DIO.ARM_STAGE_1_ENCODER_ID);
        m_stage2Encoder = new DutyCycleEncoder(DIO.ARM_STAGE_2_ENCODER_ID);
        m_stage3Encoder = new DutyCycleEncoder(DIO.ARM_STAGE_3_ENCODER_ID);

        m_stage1PID = m_stage1.getPIDController(); 
        m_stage2PID = m_stage2.getPIDController();
        m_stage3PID = m_stage3.getPIDController();

        configPID(0, 0, 0, 0, 0, 0, m_stage1Encoder, m_stage1PID);
        configPID(0, 0, 0, 0, 0, 0, m_stage2Encoder, m_stage2PID);
        configPID(0, 0, 0, 0, 0, 0, m_stage3Encoder, m_stage3PID);
    }

    private void moveToPoint(double x, double y, double claw) {
        Triangle triangle = new Triangle(x, y, Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)));
        setStage1Target(triangle.getAngleA() + (90 - Math.atan2(x, y)));
        setStage2Target(triangle.getAngleB());
        setStage3Target(claw);
    }

    private double[] forwardKinematics ( double stage1, double stage2, double stage3 ) {
        // todo forward kinematics
        return new double[2];
    }

    private double[] getCurrentPoint () {
        return forwardKinematics(m_stage1Encoder.getAbsolutePosition(), m_stage2Encoder.getAbsolutePosition(), m_stage3Encoder.getAbsolutePosition());
    }

    private void moveToAngles (double stage1Angle, double stage2Angle, double stage3Angle) {
        setStage1Target(stage1Angle);
        setStage2Target(stage2Angle);
        setStage3Target(stage3Angle);
        m_manualTargetX = getCurrentPoint()[0];
        m_manualTargetY = getCurrentPoint()[1];
        m_manualTargetTheta = getCurrentPoint()[2];
    }

    private void moveToPosition (positions position) {
        ArmPosition target = positionMap.get(position);
        m_manualTargetX = forwardKinematics(target.getStage1Angle(), target.getStage2Angle(), target.getStage3Angle())[0];
        m_manualTargetY = forwardKinematics(target.getStage1Angle(), target.getStage2Angle(), target.getStage3Angle())[1];
        m_manualTargetTheta = target.getStage3Angle();
        moveToAngles(target.getStage1Angle(), target.getStage2Angle(), target.getStage3Angle());
    }

    public Boolean isAtTarget () {
        if (RobotContainer.copilotController.getRawButton(9)) {
            return (
                Math.abs(getCurrentPoint()[0] - m_manualTargetX) < JOINT_COORDINATE_DEADZONE &&
                Math.abs(getCurrentPoint()[1] - m_manualTargetY) < JOINT_COORDINATE_DEADZONE &&
                Math.abs(getCurrentPoint()[2] - m_manualTargetTheta) < JOINT_COORDINATE_DEADZONE
            );
        } else {
            return (
                Math.abs(m_stage1Encoder.getAbsolutePosition() - m_stage1Target) < JOINT_ANGLE_DEADZONE &&
                Math.abs(m_stage2Encoder.getAbsolutePosition() - m_stage2Target) < JOINT_ANGLE_DEADZONE &&
                Math.abs(m_stage3Encoder.getAbsolutePosition() - m_stage3Target) < JOINT_ANGLE_DEADZONE
            );
        }
    }

    public void setStage1Target(double angle) {
        m_stage1Target = angle;
        m_stage1PID.setReference(angle, CANSparkMax.ControlType.kPosition);
    }

    public void setStage2Target(double angle) {
        m_stage2Target = angle;
        m_stage2PID.setReference(angle, CANSparkMax.ControlType.kPosition);
    }

    public void setStage3Target(double angle) {
        m_stage3Target = angle;
        m_stage3PID.setReference(angle, CANSparkMax.ControlType.kPosition);
    }

    public Command moveToPositionCommand (positions position) {
        m_copilotController.setLED(0, false);
        m_copilotController.setLED(1, false);
        m_copilotController.setLED(2, false);
        m_copilotController.setLED(3, false);
        m_copilotController.setLED(4, false);
        m_copilotController.setLED(5, false);

        switch (position) {
            case ScoreHigh:
                m_copilotController.setLED(2, false);
                break;
            case ScoreMid:
                m_copilotController.setLED(4, false);
                break;
            case ScoreLow:
                m_copilotController.setLED(5, false);
                break;
            case Floor:
                m_copilotController.setLED(1, false);
                break;
            case FloorAlt:
                m_copilotController.setLED(3, false);
                break;
            case Substation:
                m_copilotController.setLED(0, false);
                break;
            case Idle:
                break;
        }
        return new FunctionalCommand(
            () -> { // init
                m_clawSubsystem.notake();
            }, 
            () -> { // execution
                moveToPosition(position);
            }, 
            interrupted -> { // when should the command do when it ends?
                if (!interrupted) {
                    // arm is in position
                    if ( position == positions.Idle ) return; // idle position is exempt from driver notification
                    m_LEDsSubsystem.flashGreen().schedule();
                    m_driverController.getHID().setRumble(RumbleType.kBothRumble, 1);
                    new SequentialCommandGroup(new WaitCommand(0.5), new InstantCommand( () -> m_driverController.getHID().setRumble(RumbleType.kBothRumble, 0))).schedule();
                }
            },
            () -> { // should the command end?
                return this.isAtTarget();
            },
            this
        );
    }

    public Command moveToPointCommand (double x, double y, double theta) {
        return new FunctionalCommand(
            () -> { // init
                m_clawSubsystem.notake();
            }, 
            () -> { // execution
                moveToPoint(x, y, theta);
            }, 
            interrupted -> { // when should the command do when it ends?
                if (!interrupted) {
                    // arm is in position
                }
            },
            () -> { // should the command end?
                return this.isAtTarget();
            },
            this
        );
    }

    public Command defaultCommand () {
        if (RobotContainer.copilotController.getRawButton(9)) {
            m_copilotController.setLED(0, false);
            m_copilotController.setLED(1, false);
            m_copilotController.setLED(2, false);
            m_copilotController.setLED(3, false);
            m_copilotController.setLED(4, false);
            m_copilotController.setLED(5, false);
            m_copilotController.setLED(6, false);
            m_copilotController.setLED(7, false);
            m_copilotController.setLED(8, false);

            m_copilotController.setLED(10, true);
            m_copilotController.setLED(11, true);
            m_copilotController.setLED(12, true);
            m_copilotController.setLED(13, true);
            m_copilotController.setLED(14, true);

            // the manual override is enabled
            // TODO modify targets from buttons i cannot be bothered rn
            return moveToPointCommand(m_manualTargetX, m_manualTargetY, m_manualTargetTheta);
        } else {
            m_copilotController.setLED(10, false);
            m_copilotController.setLED(11, false);
            m_copilotController.setLED(12, false);
            m_copilotController.setLED(13, false);
            m_copilotController.setLED(14, false);
            return moveToPositionCommand(positions.Idle);
        }
    }

    @Override  public void periodic() {
        Telemetry.setValue("POP/stage1/setpoint", m_stage1.get());
        Telemetry.setValue("POP/stage1/temperature", m_stage1.getMotorTemperature());
        Telemetry.setValue("POP/stage1/outputVoltage", m_stage1.getAppliedOutput());
        Telemetry.setValue("POP/stage1/statorCurrent", m_stage1.getOutputCurrent());
        Telemetry.setValue("POP/stage1/actualPosition", m_stage1Encoder.getAbsolutePosition());
        Telemetry.setValue("POP/stage1/targetPosition", m_stage1Target);
        Telemetry.setValue("POP/stage2/setpoint", m_stage2.get());
        Telemetry.setValue("POP/stage2/temperature", m_stage2.getMotorTemperature());
        Telemetry.setValue("POP/stage2/outputVoltage", m_stage2.getAppliedOutput());
        Telemetry.setValue("POP/stage2/statorcurrent", m_stage2.getOutputCurrent());
        Telemetry.setValue("POP/stage2/actualPosition", m_stage2Encoder.getAbsolutePosition());
        Telemetry.setValue("POP/stage2/targetPosition", m_stage2Target);
        Telemetry.setValue("POP/stage3/setpoint", m_stage3.get());
        Telemetry.setValue("POP/stage3/temperature", m_stage3.getMotorTemperature());
        Telemetry.setValue("POP/stage3/outputVoltage", m_stage3.getAppliedOutput());
        Telemetry.setValue("POP/stage3/statorCurrent", m_stage3.getOutputCurrent());
        Telemetry.setValue("POP/stage3/actualPosition", m_stage3Encoder.getAbsolutePosition());
        Telemetry.setValue("POP/stage3/targetPosition", m_stage3Target);
    }
    
    @Override  public void simulationPeriodic() {}

    public void configPID(double kp, double kd, double FF, double maxV, double maxA, int profile, DutyCycleEncoder encoder, SparkMaxPIDController controller) {
        // todo redo arm PID controllers to use PIDController
        controller.setP(kp, profile);
        controller.setD(kd, profile);
        controller.setFF(FF, profile);
        controller.setSmartMotionMaxAccel(maxA, profile);
        controller.setSmartMotionMaxVelocity(maxV, profile);
        controller.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, profile);
    }
}