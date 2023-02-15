package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Telemetry;
import frc.robot.Constants;
import frc.robot.Constants.POP;

public class PinchersofPower extends SubsystemBase  {
  private final Compressor comp;
  private final DoubleSolenoid pusher;
  private final CANSparkMax spinner;
  private final CANSparkMax spinner2;
  private boolean m_cone;

  public PinchersofPower() {
    comp = new Compressor(Constants.CAN.PCH_ID, PneumaticsModuleType.CTREPCM);
    pusher = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.POP.FORWARD_PNEUMATIC_CHANNEL, Constants.POP.BACKWARD_PNEUMATIC_CHANNEL);
    spinner = new CANSparkMax(Constants.CAN.GRIP_LEFT_ID, MotorType.kBrushless);
    spinner2 = new CANSparkMax(Constants.CAN.GRIP_RIGHT_ID, MotorType.kBrushless);
    spinner2.follow(spinner);
    m_cone = false;
  }

  public void forward() {
    pusher.set(Value.kForward);
  }

  public void reverse() {
    pusher.set(Value.kReverse);
  }

  public void off() {
    pusher.set(Value.kOff);
  }

  public void spinin() {
    spinner.set(POP.SPEED);
  }

  public void spinout() {
    spinner.set(-POP.SPEED);
  }

  public void spinoff() {
    spinner.set(0);
  }

  public void enable() {
    comp.enableDigital();
  }

  public void disable() {
    comp.disable();
  }

  public void intake() {
    if((pusher.get() == Value.kForward)) {
      reverse();
    }
    if((pusher.get() != Value.kForward) && (m_cone == true)) {
      forward();
    }

    spinin();
    new WaitCommand(1).andThen(new InstantCommand(() -> notake())).schedule();
  }

  public void outtake() {
    spinout();
    if(pusher.get() == Value.kForward) {
      pusher.set(Value.kReverse);
    }
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
    return new InstantCommand(() -> outtake(), this);
  }

  public Command notakeCommand() {
    return new InstantCommand(() -> notake(), this);
  }

  @Override
  public void periodic() {
    Telemetry.setValue("POP/leftMotor/setpoint", spinner.get());
    Telemetry.setValue("POP/leftMotor/temperature", spinner.getMotorTemperature());
    Telemetry.setValue("POP/leftMotor/outputVoltage", spinner.getAppliedOutput());
    Telemetry.setValue("POP/leftMotor/statorcurrent", spinner.getOutputCurrent());
    Telemetry.setValue("POP/rightMotor/setpoint", spinner2.get());
    Telemetry.setValue("POP/rightMotor/temperature", spinner2.getMotorTemperature());
    Telemetry.setValue("POP/rightMotor/outputVoltage", spinner2.getAppliedOutput());
    Telemetry.setValue("POP/rightMotor/statorCurrent", spinner2.getOutputCurrent());
    Telemetry.setValue("POP/piston", pusher.get());
  }
}