package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.FrostConfigs;
import frc.lib.Telemetry;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.POP;
import frc.robot.Constants.ARM.positions;
import java.util.function.Supplier;

public class PinchersofPower extends SubsystemBase  {
  private final Compressor comp;
  private final DoubleSolenoid pusher;
  private final CANSparkMax spinner;
  private final CANSparkMax spinner2;
  private boolean m_cone;
  private double intakeSpeed = 0;
  private DigitalInput limitSwitch = new DigitalInput(Constants.DIO.GRIP_LIMIT_SWITCH);
  private Supplier<positions> armPos;

  public PinchersofPower() {
    comp = new Compressor(Constants.CAN.PCH_ID, PneumaticsModuleType.REVPH);
    pusher = new DoubleSolenoid(Constants.CAN.PCH_ID, PneumaticsModuleType.REVPH, Constants.POP.FORWARD_PNEUMATIC_CHANNEL, Constants.POP.BACKWARD_PNEUMATIC_CHANNEL);

    spinner = new CANSparkMax(Constants.CAN.GRIP_LEFT_ID, MotorType.kBrushless);
    spinner2 = new CANSparkMax(Constants.CAN.GRIP_RIGHT_ID, MotorType.kBrushless);

    m_cone = true;

    FrostConfigs.configIntakeMotor(spinner, false);
    FrostConfigs.configIntakeMotor(spinner, true);

    armPos = () -> positions.Idle;
  }

  public void setArmPos(Supplier<positions> armPos) {
    this.armPos = armPos;
  }

  public boolean getGPDetected(){
    return !limitSwitch.get();
  }

  public void closeGrip() {
    pusher.set(Value.kForward);
  }

  public void openGrip() {
    pusher.set(Value.kReverse);
  }

  public void toggle () {
    if (pusher.get() == Value.kReverse) pusher.set(Value.kForward);
    else pusher.set(Value.kReverse);
  }

  public void spinSlow() {
    intakeSpeed = 0.15;//0.2; 
  }

  public void spinIn() {
    intakeSpeed = POP.SPEEDIN;
  }

  public void spinOut() {
    intakeSpeed = -POP.SPEEDOUT;
  }

  public void spinOff() {
    intakeSpeed = 0;
  }

  public enum GamePieces {
    Cone,
    Cube,
    None
  }

  public void intake() {
    if(!m_cone) openGrip();
    else closeGrip();
  }
  public void setCone(boolean check) { m_cone = check; }

  public boolean wantCone () { return m_cone; }

  public void setMode(GamePieces mode) {
    m_cone = (mode == GamePieces.Cone);
    spinSlow();

    if(!m_cone) openGrip();
  }

  public Command intakeCommand() {
    return new InstantCommand(() -> intake(), this);
  }

  public Command outTakeCommand() {
    return new InstantCommand( () -> {
      if ( armPos.get() == positions.Substation && m_cone) { closeGrip();} 
      else if ( m_cone ) {
        if ( armPos.get() == positions.ScoreLow) spinOut();  
        spinOff();
        openGrip();
      } 
      else spinOut();
    });
  }

  public Command spinOffCommand() { return new InstantCommand(() -> spinOff(), this); }

  public boolean setClawAndIdleState(positions position) {
    switch (position) {
      case ScoreHighCone:
      case ScoreHighCube:
      case ScoreMidCone:
      case ScoreMidCube:
      case ScoreLow:
        return false;
      case Floor:
      case FloorAlt:
      case Substation:
          spinIn();
          openGrip();
          return false;
      case Idle:
          spinSlow();
          return true;
      default:
          spinSlow();
          return false;
    }
  }

  @Override
  public void periodic() {
    if ( DriverStation.isEnabled() || DriverStation.isAutonomousEnabled() ) {
      spinner.set(intakeSpeed);
      spinner2.set(intakeSpeed);

      if ( getGPDetected() && (armPos.get() == positions.Substation || armPos.get() == positions.Floor) 
      && m_cone && !RobotContainer.copilotController.getRawButton(15) ) closeGrip();
    } else {
      // prevent CAN timeouts when disabled, actual motor stoppage is handled at a lower level
      spinner.set(0);
      spinner2.set(0);
    }

    intakeMotorTelemetry("leftMotor", spinner);
    intakeMotorTelemetry("rightMotor", spinner2);
    
    Telemetry.setValue("Pincher/limitSwitch", !limitSwitch.get());
    Telemetry.setValue("Pincher/piston", pusher.get() == DoubleSolenoid.Value.kForward ? "Forward" : "Reverse");
    Telemetry.setValue("Pincher/compressor/pressure", comp.getPressure());
  }

  public void intakeMotorTelemetry(String key, CANSparkMax motor) {
    Telemetry.setValue("Pincher/"+key+"/setpoint", motor.get());
    Telemetry.setValue("Pincher/"+key+"/temperature", motor.getMotorTemperature());
    Telemetry.setValue("Pincher/"+key+"/outputVoltage", motor.getAppliedOutput());
    Telemetry.setValue("Pincher/"+key+"/statorcurrent", motor.getOutputCurrent());
  }
}