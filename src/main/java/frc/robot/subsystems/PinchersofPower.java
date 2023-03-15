package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Telemetry;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.POP;

public class PinchersofPower extends SubsystemBase  {
  private final Compressor comp;
  private final DoubleSolenoid pusher;
  private final CANSparkMax spinner;
  private final CANSparkMax spinner2;
  private final ColorSensorV3 colorSensor;
  private boolean m_cone;

  public PinchersofPower() {
    comp = new Compressor(Constants.CAN.PCH_ID, PneumaticsModuleType.REVPH);
    pusher = new DoubleSolenoid(Constants.CAN.PCH_ID, PneumaticsModuleType.REVPH, Constants.POP.FORWARD_PNEUMATIC_CHANNEL, Constants.POP.BACKWARD_PNEUMATIC_CHANNEL);
    spinner = new CANSparkMax(Constants.CAN.GRIP_LEFT_ID, MotorType.kBrushless);
    spinner2 = new CANSparkMax(Constants.CAN.GRIP_RIGHT_ID, MotorType.kBrushless);
    //spinner2.follow(spinner, true);
    colorSensor = new ColorSensorV3(I2C.Port.kMXP);
    m_cone = true;
    pusher.set(Value.kForward);

    //comp.disable();

    spinner.restoreFactoryDefaults();
    spinner2.restoreFactoryDefaults();

    spinner.clearFaults();
    spinner2.clearFaults();

    spinner.setIdleMode(IdleMode.kBrake);
    spinner2.setIdleMode(IdleMode.kBrake);

    spinner.setSmartCurrentLimit(40);
    spinner2.setSmartCurrentLimit(40);

    spinner2.setInverted(true);

    spinner.burnFlash();
    spinner2.burnFlash();

  }

  /** Close */
  public void forward() {
    pusher.set(Value.kForward);
  }

  /** Open */
  public void reverse() {
    pusher.set(Value.kReverse);
  }

  public void off() {
    pusher.set(Value.kOff);
  }

  public void toggle () {
    if (pusher.get() == Value.kReverse) {
      pusher.set(Value.kForward);
    } else {
      pusher.set(Value.kReverse);
    }
  }

  public void spinin() {
    spinner.set(POP.SPEED);
    spinner2.set(POP.SPEED);
  }

  public void spinout() {
    spinner.set(-POP.SPEED);
    spinner2.set(-POP.SPEED);
  }

  public void spinoff() {
    spinner.set(0);
    spinner2.set(0);

  }

  public void enable() {
    comp.enableDigital();
  }

  public void disable() {
    comp.disable();
  }

  public enum GamePieces {
    Cone,
    Cube,
    None
  }

  public GamePieces whatGamePieceIsTheIntakeHoldingAtTheCurrentMoment () {
    Color actualColor = colorSensor.getColor();
    if ( colorSensor.getProximity() > 100 ) {
      if (actualColor.green < actualColor.blue) { // cube
        return GamePieces.Cube;
      } else if (actualColor.green < actualColor.blue) { // cone
        return GamePieces.Cone;
      } else {
        return GamePieces.None;
      }
    } else {
      return GamePieces.None;
    }
  }

  public void intake() {
    if (m_cone || whatGamePieceIsTheIntakeHoldingAtTheCurrentMoment() == GamePieces.Cone) {
      forward();
    } else {
      spinin();
    }
  }

  public boolean wantCone () {
    return m_cone;
  }

  public boolean isOpen() {
    if ( whatGamePieceIsTheIntakeHoldingAtTheCurrentMoment() == GamePieces.Cone || m_cone ) {
      return pusher.get() == Value.kReverse;
    } else if ( whatGamePieceIsTheIntakeHoldingAtTheCurrentMoment() == GamePieces.Cube ) {
      return false;
    } else {
      return false;
    }
  }

  public void outtake() {
    spinout();

  }

  public void notake() {
    spinoff();
  }

  public void setMode(String mode) {
    if(mode == "cone") {
      m_cone = true;
    }
    if(mode == "cube") {
      m_cone = false;
    }
  }

  public Command intakeCommand() {
    return new InstantCommand(() -> intake(), this);
  }

  public Command outtakeCommand() {
    if ( m_cone ) {
      return new InstantCommand(() -> spinout(), this).andThen(new InstantCommand( () -> reverse()));
    }
    return new InstantCommand(() -> spinout(), this);
  }

  public Command notakeCommand() {
    return new InstantCommand(() -> notake(), this);
  }

  @Override
  public void periodic() {
    Telemetry.setValue("Pincher/leftMotor/setpoint", spinner.get());
    Telemetry.setValue("Pincher/leftMotor/temperature", spinner.getMotorTemperature());
    Telemetry.setValue("Pincher/leftMotor/outputVoltage", spinner.getAppliedOutput());
    Telemetry.setValue("Pincher/leftMotor/statorcurrent", spinner.getOutputCurrent());
    Telemetry.setValue("Pincher/rightMotor/setpoint", spinner2.get());
    Telemetry.setValue("Pincher/rightMotor/temperature", spinner2.getMotorTemperature());
    Telemetry.setValue("Pincher/rightMotor/outputVoltage", spinner2.getAppliedOutput());
    Telemetry.setValue("Pincher/rightMotor/statorCurrent", spinner2.getOutputCurrent());
    Telemetry.setValue("Pincher/piston", pusher.get() == DoubleSolenoid.Value.kForward ? "Forward" : "Reverse");
    Telemetry.setValue("Pincher/compressor/pressure", comp.getPressure());

    if ( !RobotContainer.copilotController.getRawButton(9) && m_cone ) {
      spinoff();
    }
  }
}