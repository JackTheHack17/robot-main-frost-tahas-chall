package frc.robot.subsystems;

import static frc.robot.Constants.ARM.JOINT_ANGLE_DEADZONE;
import static frc.robot.Constants.ARM.JOINT_COORDINATE_DEADZONE;
import static frc.robot.Constants.ARM.STAGE_1_Kd;
import static frc.robot.Constants.ARM.STAGE_1_Ki;
import static frc.robot.Constants.ARM.STAGE_1_Kp;
import static frc.robot.Constants.ARM.STAGE_1_LENGTH;
import static frc.robot.Constants.ARM.STAGE_1_OFFSET;
import static frc.robot.Constants.ARM.STAGE_2_Kd;
import static frc.robot.Constants.ARM.STAGE_2_Ki;
import static frc.robot.Constants.ARM.STAGE_2_Kp;
import static frc.robot.Constants.ARM.STAGE_2_LENGTH;
import static frc.robot.Constants.ARM.STAGE_2_OFFSET;
import static frc.robot.Constants.ARM.STAGE_3_Kd;
import static frc.robot.Constants.ARM.STAGE_3_Ki;
import static frc.robot.Constants.ARM.STAGE_3_Kp;
import static frc.robot.Constants.ARM.STAGE_3_OFFSET;
import static frc.robot.Constants.ARM.floorAltPosition;
import static frc.robot.Constants.ARM.floorPosition;
import static frc.robot.Constants.ARM.idlePosition;
import static frc.robot.Constants.ARM.scoreHighPosition;
import static frc.robot.Constants.ARM.scoreLowPosition;
import static frc.robot.Constants.ARM.scoreMidPosition;
import static frc.robot.Constants.ARM.substationPosition;
import static frc.robot.Constants.ARM.thetaSpeed;
import static frc.robot.Constants.ARM.xSpeed;
import static frc.robot.Constants.ARM.ySpeed;

import java.util.HashMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.ArmPosition;
import frc.lib.ButtonBoard;
import frc.lib.Telemetry;
import frc.lib.Triangle;
import frc.robot.Constants.ARM.positions;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DIO;
import frc.robot.RobotContainer;
import frc.robot.subsystems.PinchersofPower.GamePieces;

public class Arm extends SubsystemBase {
    private final CANSparkMax m_stage1;
    private final CANSparkMax m_stage2;
    private final CANSparkMax m_stage3;
    private final DutyCycleEncoder m_stage1Encoder;  
    private final DutyCycleEncoder m_stage2Encoder;
    private final DutyCycleEncoder m_stage3Encoder;
    private final PIDController m_stage1PID;  
    private final PIDController m_stage2PID;
    private final PIDController m_stage3PID;
    private PinchersofPower m_clawSubsystem;
    private ButtonBoard m_copilotController;
    private double m_stage1Target = 0;
    private double m_stage2Target = 0;
    private double m_stage3Target = 0;
    // manual targets actually unused
    private double m_manualTargetX = 0;
    private double m_manualTargetY = 0;
    private double m_manualTargetTheta = 0;
    private HashMap<positions, ArmPosition> positionMap = new HashMap<positions, ArmPosition>();
    private boolean movingToIdle = false;
    private positions target = positions.Idle;
    private positions lastPosition;
    private ArmPosition fromIdleTargetPosition;
    private ArmFeedforward m_stage1FF;
    private ArmFeedforward m_stage2FF;
    private ArmFeedforward m_stage3FF;
    private boolean returnToIdle = true;

    public Arm(PinchersofPower m_claw, ButtonBoard copilotController) {
        // populate position map
        positionMap.put(positions.ScoreHigh, scoreHighPosition);
        positionMap.put(positions.ScoreMid, scoreMidPosition);
        positionMap.put(positions.ScoreLow, scoreLowPosition);
        positionMap.put(positions.Floor, floorPosition);
        positionMap.put(positions.FloorAlt, floorAltPosition);
        positionMap.put(positions.Substation, substationPosition);
        positionMap.put(positions.Idle, idlePosition);

        m_clawSubsystem = m_claw;
        m_copilotController = copilotController;

        m_stage1 = new CANSparkMax(CAN.ARM_STAGE_1_ID, MotorType.kBrushless);
        m_stage2 = new CANSparkMax(CAN.ARM_STAGE_2_ID, MotorType.kBrushless);
        m_stage3 = new CANSparkMax(CAN.ARM_STAGE_3_ID, MotorType.kBrushless);

        m_stage1.setIdleMode(IdleMode.kBrake);
        m_stage2.setIdleMode(IdleMode.kBrake);
        m_stage3.setIdleMode(IdleMode.kBrake);

        m_stage2.setInverted(false);
        m_stage3.setInverted(false);

        m_stage1Encoder = new DutyCycleEncoder(DIO.ARM_STAGE_1_ENCODER_ID);
        m_stage2Encoder = new DutyCycleEncoder(DIO.ARM_STAGE_2_ENCODER_ID);
        m_stage3Encoder = new DutyCycleEncoder(DIO.ARM_STAGE_3_ENCODER_ID);

        m_stage1FF = new ArmFeedforward(0.04, 1.1,0,0.07);
        m_stage2FF = new ArmFeedforward(0.04, 0.92, 0,0.13);
        m_stage3FF = new ArmFeedforward(0.04, 0.52, 1.95,0.04);

        m_stage1PID = new PIDController(STAGE_1_Kp, STAGE_1_Ki, STAGE_1_Kd);
        m_stage1PID.enableContinuousInput(0, 360);
        m_stage2PID = new PIDController(STAGE_2_Kp, STAGE_2_Ki, STAGE_2_Kd);
        m_stage2PID.enableContinuousInput(0, 360);
        m_stage3PID = new PIDController(STAGE_3_Kp, STAGE_3_Ki, STAGE_3_Kd);
        m_stage3PID.enableContinuousInput(0, 360);

        m_stage1PID.setTolerance(0);
        m_stage2PID.setTolerance(1.5);
        m_stage3PID.setTolerance(0);

        m_stage1.restoreFactoryDefaults();
        m_stage1.clearFaults();
        m_stage1.setSmartCurrentLimit(40);
        m_stage1.setSecondaryCurrentLimit(40);
        m_stage1.burnFlash();

        m_stage2.restoreFactoryDefaults();
        m_stage2.clearFaults();
        m_stage2.setSmartCurrentLimit(40);
        m_stage2.setSecondaryCurrentLimit(40);
        m_stage2.burnFlash();

        m_stage3.restoreFactoryDefaults();
        m_stage3.clearFaults();
        m_stage3.setSmartCurrentLimit(40);
        m_stage3.setSecondaryCurrentLimit(40);
        m_stage3.burnFlash();

        m_copilotController.button(10).whileTrue(new RepeatCommand( new InstantCommand(() -> {
            if (m_copilotController.getRawButton(9)) m_manualTargetTheta += thetaSpeed;
        })));
        m_copilotController.button(11).whileTrue(new RepeatCommand( new InstantCommand(() -> {
            if (m_copilotController.getRawButton(9)) m_manualTargetTheta -= thetaSpeed;
        })));

        lastPosition = positions.Idle;
    }

    private double[] inverseKinematics (double x, double y, double theta) {
        double[] output = new double[3];

        double l1 = STAGE_1_LENGTH;
        double l2 = STAGE_2_LENGTH;
        double l3 = Math.hypot(x, y);
        double thetaA = Math.toDegrees(Math.acos((l2*l2-l1*l1-l3*l3)/(-2*l1*l3)));
        double thetaB = Math.toDegrees(Math.acos((l3*l3-l1*l1-l2*l2)/(-2*l1*l2)));

        output[0] = (360 + Math.toDegrees(Math.atan2(y, x)) + thetaA) % 360;
        output[1] = (360 + output[0] + thetaB + 180) % 360;
        output[2] = (360 + theta) % 360;

        if ( y >= 20 || x > 49.0 || output[0] != output[0] || output[1] != output[1] || output[2] != output[2])  {
            output = getCurrentPoint();
        }

        return output;
    }

    private void moveToPoint (double x, double y, double theta) {
        double[] thetas = inverseKinematics(x, y, theta);
        moveToAngles(thetas[0], thetas[1], thetas[2]);
    }

    public void toggleIdle () {
        returnToIdle = !returnToIdle;
        defaultCommand().schedule();
    }

    /** all inputs in radians */
    private double[] forwardKinematics ( double stage1, double stage2, double stage3 ) {
        double[] output = new double[3];
        output[0] = Math.cos(Math.toRadians(stage1 - STAGE_1_OFFSET)) * STAGE_1_LENGTH + Math.cos(Math.toRadians(stage2 - STAGE_2_OFFSET)) * (STAGE_2_LENGTH);
        output[1] = Math.sin(Math.toRadians(stage1 - STAGE_1_OFFSET)) * STAGE_1_LENGTH + Math.sin(Math.toRadians(stage2 - STAGE_2_OFFSET)) * (STAGE_2_LENGTH);
        output[2] = stage3 - STAGE_3_OFFSET;
        //ArmPosition armpos = new ArmPosition(stage1, stage2, stage3);
        //double[] output = {armpos.getXPosition(), armpos.getYPosition(), stage3};
        return output;
    }

    private double[] getCurrentPoint () {
        return forwardKinematics(m_stage1Encoder.getAbsolutePosition()*360, m_stage2Encoder.getAbsolutePosition()*360, m_stage3Encoder.getAbsolutePosition()*360);
    }

    private void moveToAngles (double stage1Angle, double stage2Angle, double stage3Angle) {
        m_manualTargetX = forwardKinematics(stage1Angle - STAGE_1_OFFSET, stage2Angle - STAGE_2_OFFSET, stage3Angle - STAGE_3_OFFSET)[0];
        m_manualTargetY = forwardKinematics(stage1Angle - STAGE_1_OFFSET, stage2Angle - STAGE_2_OFFSET, stage3Angle - STAGE_3_OFFSET)[1];
        m_manualTargetTheta = stage3Angle - STAGE_3_OFFSET;
        setStage1Target(stage1Angle);
        setStage2Target(stage2Angle);
        setStage3Target(stage3Angle);
    }

    private void moveToPosition (positions position) {
        target = position;
        ArmPosition target = positionMap.get(position);
        fromIdleTargetPosition = target;
        if ( lastPosition == positions.Idle && position != positions.Idle && Math.abs(m_stage2Encoder.getAbsolutePosition()*360 - target.getStage2Angle()) > JOINT_ANGLE_DEADZONE) {
            moveToAngles(m_stage1Encoder.getAbsolutePosition()*360, target.getStage2Angle(), target.getStage3Angle());
            //m_manualTargetX = forwardKinematics(m_stage1Encoder.getAbsolutePosition()*360, target.getStage2Angle(), target.getStage3Angle())[0];
            //m_manualTargetY = forwardKinematics(m_stage1Encoder.getAbsolutePosition()*360, target.getStage2Angle(), target.getStage3Angle())[1];
            //m_manualTargetTheta = target.getStage3Angle();
        } else if ( position == positions.Idle && !movingToIdle && Math.abs(m_stage1Encoder.getAbsolutePosition()*360 - target.getStage1Angle()) > JOINT_ANGLE_DEADZONE ) {
            moveToAngles(target.getStage1Angle(), m_stage2Encoder.getAbsolutePosition()*360, target.getStage3Angle());
            movingToIdle = true;
            //m_manualTargetX = forwardKinematics(target.getStage1Angle(), m_stage2Encoder.getAbsolutePosition()*360, target.getStage3Angle())[0];
            //m_manualTargetY = forwardKinematics(target.getStage1Angle(), m_stage2Encoder.getAbsolutePosition()*360, target.getStage3Angle())[1];
            //m_manualTargetTheta = target.getStage3Angle();
        } else if ( !movingToIdle && m_stage1Encoder.getAbsolutePosition()*360 - m_stage1Target < JOINT_ANGLE_DEADZONE ) {
            //m_manualTargetX = forwardKinematics(target.getStage1Angle(), target.getStage2Angle(), target.getStage3Angle())[0];
            //m_manualTargetY = forwardKinematics(target.getStage1Angle(), target.getStage2Angle(), target.getStage3Angle())[1];
            //m_manualTargetTheta = target.getStage3Angle();
            moveToAngles(target.getStage1Angle(), target.getStage2Angle(), target.getStage3Angle());
        }
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
                Math.abs(m_stage1Encoder.getAbsolutePosition()*360 - m_stage1Target) < JOINT_ANGLE_DEADZONE &&
                Math.abs(m_stage2Encoder.getAbsolutePosition()*360 - m_stage2Target) < JOINT_ANGLE_DEADZONE &&
                Math.abs(m_stage3Encoder.getAbsolutePosition()*360 - m_stage3Target) < JOINT_ANGLE_DEADZONE
            );
        }
    }

    public void setStage1Target(double angle) {
        m_stage1Target = angle;
    }

    public void setStage2Target(double angle) {
        m_stage2Target = angle;
    }

    public void setStage3Target(double angle) {
        m_stage3Target = angle;
    }

    public Command moveToPositionCommand (positions position) {
        return new FunctionalCommand(
            () -> { // init
                if (!m_copilotController.getRawButton(9)) {
                    m_copilotController.setLED(10, false);
                    m_copilotController.setLED(11, false);
                    m_copilotController.setLED(12, false);
                    m_copilotController.setLED(13, false);
                    m_copilotController.setLED(14, false);
                }
                m_copilotController.setLED(0, false);
                m_copilotController.setLED(1, false);
                m_copilotController.setLED(2, false);
                m_copilotController.setLED(3, false);
                m_copilotController.setLED(4, false);
                m_copilotController.setLED(5, false);
                
                m_clawSubsystem.spinoff();
        
                switch (position) {
                    case ScoreHigh:
                        m_copilotController.setLED(2, true);
                        break;
                    case ScoreMid:
                        m_copilotController.setLED(4, true);
                        break;
                    case ScoreLow:
                        m_copilotController.setLED(5, true);
                        break;
                    case Floor:
                        if ( !m_clawSubsystem.wantCone() ) m_clawSubsystem.spinin();
                        m_clawSubsystem.reverse();
                        m_copilotController.setLED(1, true);
                        break;
                    case FloorAlt:
                        if ( !m_clawSubsystem.wantCone() ) m_clawSubsystem.spinin();
                        m_clawSubsystem.reverse();
                        m_copilotController.setLED(3, true);
                        break;
                    case Substation:
                        if ( !m_clawSubsystem.wantCone() ) m_clawSubsystem.spinin();
                        m_clawSubsystem.reverse();
                        m_copilotController.setLED(0, true);
                        break;
                    case Idle:
                    case IdleShootPosition:
                        break;
                    default:
                        break;
                }
            }, 
            () -> { // execution
                //if ( !m_clawSubsystem.wantCone() && position != positions.Idle && m_clawSubsystem.whatGamePieceIsTheIntakeHoldingAtTheCurrentMoment() == GamePieces.None) m_clawSubsystem.spinin();
                moveToPosition(position);
            }, 
            interrupted -> { // when should the command do when it ends?
                lastPosition = position;
                movingToIdle = false;
            },
            () -> { // should the command end?
                //return this.isAtTarget();
                return false;
            },
            this
        );
    }

    public Command moveToPositionTerminatingCommand( positions position ) {
        return new FunctionalCommand(
            () -> { // init
                if (!m_copilotController.getRawButton(9)) {
                    m_copilotController.setLED(10, false);
                    m_copilotController.setLED(11, false);
                    m_copilotController.setLED(12, false);
                    m_copilotController.setLED(13, false);
                    m_copilotController.setLED(14, false);
                }
                m_copilotController.setLED(0, false);
                m_copilotController.setLED(1, false);
                m_copilotController.setLED(2, false);
                m_copilotController.setLED(3, false);
                m_copilotController.setLED(4, false);
                m_copilotController.setLED(5, false);
        
                switch (position) {
                    case ScoreHigh:
                        m_copilotController.setLED(2, true);
                        break;
                    case ScoreMid:
                        m_copilotController.setLED(4, true);
                        break;
                    case ScoreLow:
                        m_copilotController.setLED(5, true);
                        break;
                    case Floor:
                        m_clawSubsystem.reverse();
                        m_copilotController.setLED(1, true);
                        break;
                    case FloorAlt:
                        m_clawSubsystem.reverse();
                        m_copilotController.setLED(3, true);
                        break;
                    case Substation:
                        m_clawSubsystem.reverse();
                        m_copilotController.setLED(0, true);
                        break;
                    case Idle:
                    case IdleShootPosition:
                        break;
                    default:
                        break;
                }
            }, 
            () -> { // execution
                moveToPosition(position);
            }, 
            interrupted -> { // when should the command do when it ends?
                lastPosition = position;
                movingToIdle = false;
            },
            () -> { // should the command end?
                return this.isAtTarget();
            },
            this
        );
    }

    public Command placeCommand () {
        if (!m_clawSubsystem.wantCone() || m_clawSubsystem.whatGamePieceIsTheIntakeHoldingAtTheCurrentMoment() != GamePieces.Cone) return new InstantCommand();
        switch ( target ) {
            case ScoreHigh:
                return moveToPositionTerminatingCommand(positions.ScoreHighPlace);
            case ScoreMid:
                return moveToPositionTerminatingCommand(positions.ScoreMidPlace);
            default:
                return new InstantCommand();
        }
    }

    public Command moveToPointCommand () {
        return new FunctionalCommand(
            () -> { // init
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
            }, 
            () -> { // execution
                m_manualTargetX += m_copilotController.getJoystick().getX() * xSpeed;
                m_manualTargetY += -m_copilotController.getJoystick().getY() * ySpeed;
                moveToPoint(m_manualTargetX, m_manualTargetY, m_manualTargetTheta);
                //moveToPoint(m_stage1Encoder.getAbsolutePosition()*360 - STAGE_1_OFFSET, m_stage2Encoder.getAbsolutePosition()*360, m_stage3Encoder.getAbsolutePosition()*360 - STAGE_2_OFFSET, m_copilotController.getJoystick().getX() * xSpeed, m_copilotController.getJoystick().getY() * ySpeed);
            }, 

            interrupted -> { // when should the command do when it ends?
                if (!interrupted) {
                    // arm is in position
                }
            },
            () -> { // should the command end?
                return true; //this.isAtTarget();
            },
            this
        );
    }

    public Command defaultCommand () {
        return new FunctionalCommand(
            () -> {},
            () -> {
                if (RobotContainer.copilotController.getRawButton(9)) {
                    moveToPointCommand().schedule();
                } else {
                    if ( !m_clawSubsystem.isOpen() ) {
                        if (returnToIdle) {
                            moveToPositionCommand(positions.Idle).schedule();
                        } else {
                            moveToPositionCommand(positions.IdleShootPosition).schedule();
                        }
                    }
                }
            }, 
            (interrupted) -> {},
            () -> false, 
            (Subsystem) this
        );
    }

    @Override  public void periodic() {
        Telemetry.setValue("Arm/targetPosition", target.name());
        Telemetry.setValue("Arm/currentPoint/x", getCurrentPoint()[0]);
        Telemetry.setValue("Arm/currentPoint/y", getCurrentPoint()[1]);
        Telemetry.setValue("Arm/currentPoint/theta", getCurrentPoint()[2]);
        Telemetry.setValue("Arm/manualTarget/manualTargetX", m_manualTargetX);
        Telemetry.setValue("Arm/manualTarget/manualTargetY", m_manualTargetY);
        Telemetry.setValue("Arm/manualTarget/manualTargetTheta", m_manualTargetTheta);
        double[] telemetryConversion = inverseKinematics(m_manualTargetX, m_manualTargetY, m_manualTargetTheta);
        Telemetry.setValue("Arm/manualTarget/manualTargetFK", forwardKinematics(telemetryConversion[0], telemetryConversion[1], telemetryConversion[2]));
        Telemetry.setValue("Arm/stage1/setpoint", m_stage1.get());
        Telemetry.setValue("Arm/stage1/temperature", m_stage1.getMotorTemperature());
        Telemetry.setValue("Arm/stage1/outputVoltage", m_stage1.getAppliedOutput());
        Telemetry.setValue("Arm/stage1/statorCurrent", m_stage1.getOutputCurrent());
        Telemetry.setValue("Arm/stage1/actualPosition", m_stage1Encoder.getAbsolutePosition()*360);
        Telemetry.setValue("Arm/stage1/actualPositionOffset", m_stage1Encoder.getAbsolutePosition()*360 - STAGE_1_OFFSET);
        Telemetry.setValue("Arm/stage1/targetPosition", m_stage1Target);
        Telemetry.setValue("Arm/stage2/setpoint", m_stage2.get());
        Telemetry.setValue("Arm/stage2/temperature", m_stage2.getMotorTemperature());
        Telemetry.setValue("Arm/stage2/outputVoltage", m_stage2.getAppliedOutput());
        Telemetry.setValue("Arm/stage2/statorcurrent", m_stage2.getOutputCurrent());
        Telemetry.setValue("Arm/stage2/actualPosition", m_stage2Encoder.getAbsolutePosition()*360);
        Telemetry.setValue("Arm/stage2/actualPositionOffset", m_stage2Encoder.getAbsolutePosition()*360 - STAGE_2_OFFSET);
        Telemetry.setValue("Arm/stage2/targetPosition", m_stage2Target);
        Telemetry.setValue("Arm/stage3/setpoint", m_stage3.get());
        Telemetry.setValue("Arm/stage3/temperature", m_stage3.getMotorTemperature());
        Telemetry.setValue("Arm/stage3/outputVoltage", m_stage3.getAppliedOutput());
        Telemetry.setValue("Arm/stage3/statorCurrent", m_stage3.getOutputCurrent());
        Telemetry.setValue("Arm/stage3/actualPosition", m_stage3Encoder.getAbsolutePosition()*360);
        Telemetry.setValue("Arm/stage3/actualPositionOffset", m_stage3Encoder.getAbsolutePosition()*360 - STAGE_3_OFFSET);
        Telemetry.setValue("Arm/stage3/targetPosition", m_stage3Target);
        Telemetry.setValue("Arm/stage2/internalVelocity", m_stage2.getEncoder().getVelocity());

        if ( DriverStation.isEnabled() ) {
            if (movingToIdle && !m_copilotController.getRawButton(9) ) {
                if ( Math.abs(m_stage1Encoder.getAbsolutePosition()*360 - m_stage1Target) < JOINT_ANGLE_DEADZONE ) {
                    setStage2Target(idlePosition.getStage2Angle());
                    movingToIdle = false;
                }
            }
            if ( fromIdleTargetPosition != null && lastPosition == positions.Idle  && !m_copilotController.getRawButton(9) ) {
                if ( Math.abs(m_stage2Encoder.getAbsolutePosition()*360 - m_stage2Target) < JOINT_ANGLE_DEADZONE ) {
                    setStage1Target(fromIdleTargetPosition.getStage1Angle());
                }
            }
            
            //Telemetry.setValue( "Arm/stage1/theoreticalOutput", m_stage1FF.calculate(Math.toRadians(m_stage1Target - STAGE_1_OFFSET), (m_stage1.getEncoder().getVelocity()*2*Math.PI)/6000) + 12.0*MathUtil.clamp(m_stage1PID.calculate(m_stage1Encoder.getAbsolutePosition()*360 - STAGE_1_OFFSET, m_stage1Target - STAGE_1_OFFSET), -1, 1));
            //Telemetry.setValue( "Arm/stage2/theoreticalOutput", m_stage2FF.calculate(Math.toRadians(m_stage2Target - STAGE_2_OFFSET), (m_stage2.getEncoder().getVelocity()*2*Math.PI)/6000) + 12.0*MathUtil.clamp(m_stage2PID.calculate(m_stage2Encoder.getAbsolutePosition()*360 - STAGE_2_OFFSET, m_stage2Target - STAGE_2_OFFSET), -1, 1));
            //Telemetry.setValue( "Arm/stage3/theoreticalOutput", m_stage3FF.calculate(Math.toRadians(m_stage3Target - STAGE_3_OFFSET), (m_stage3.getEncoder().getVelocity()*2*Math.PI)/6000) + 12.0*MathUtil.clamp(m_stage3PID.calculate(m_stage3Encoder.getAbsolutePosition()*360 - STAGE_3_OFFSET, m_stage3Target - STAGE_3_OFFSET), -1, 1));

            if ( m_stage1Target == 0.0 && m_stage2Target == 0.0 && m_stage3Target == 0.0 ) {
                m_stage1.setVoltage(0);
                m_stage2.setVoltage(0);
                m_stage3.setVoltage(0);
                return;
            }

            m_stage1.setVoltage( m_stage1FF.calculate(Math.toRadians(m_stage1Target - STAGE_1_OFFSET), (m_stage1.getEncoder().getVelocity()*2*Math.PI)/6000) + 12.0*MathUtil.clamp(m_stage1PID.calculate(m_stage1Encoder.getAbsolutePosition()*360 - STAGE_1_OFFSET, m_stage1Target - STAGE_1_OFFSET), -1, 1));
            m_stage2.setVoltage( m_stage2FF.calculate(Math.toRadians(m_stage2Target - STAGE_2_OFFSET), (m_stage2.getEncoder().getVelocity()*2*Math.PI)/6000) + 12.0*MathUtil.clamp(m_stage2PID.calculate(m_stage2Encoder.getAbsolutePosition()*360 - STAGE_2_OFFSET, m_stage2Target - STAGE_2_OFFSET), -1, 1));
            m_stage3.setVoltage( m_stage3FF.calculate(Math.toRadians(m_stage3Target - STAGE_3_OFFSET), (m_stage3.getEncoder().getVelocity()*2*Math.PI)/6000) + 12.0*MathUtil.clamp(m_stage3PID.calculate(m_stage3Encoder.getAbsolutePosition()*360 - STAGE_3_OFFSET, m_stage3Target - STAGE_3_OFFSET), -1, 1));
        }
    }
    
    @Override  public void simulationPeriodic() {}
}