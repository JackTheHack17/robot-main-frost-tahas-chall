package frc.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.Telemetry;
import frc.robot.Constants;

// https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf

public class LEDs extends SubsystemBase {
  PWMSparkMax LEDsOutput = new PWMSparkMax(Constants.PWM.BLINKIN_ID);
  double command = 0;
  double lastColor = 0;

  public LEDs () {
    command = 0;
  }

  public Command turnYellow () {
    return new InstantCommand( () -> command = 0.69 );
  }

  public Command turnPurple () {
    return new InstantCommand( () -> command = 0.91 );
  }

  public Command flashRed () {
    lastColor = LEDsOutput.get();
    return new ParallelCommandGroup(
      new InstantCommand( () -> command = -0.11 ), 
      new SequentialCommandGroup(
        new WaitCommand(1), 
        new InstantCommand( () -> {
          if (lastColor == 0.69) {
            turnYellow().schedule();
          } else if (lastColor == 0.91) {
            turnPurple().schedule();
          } else {
            idle().schedule();
          }
        })
      )
    );
  }

  public Command flashGreen () {
    lastColor = LEDsOutput.get();
    return new ParallelCommandGroup(
      new InstantCommand( () -> command = 0.15 ), 
      new SequentialCommandGroup(
        new WaitCommand(1), 
        new InstantCommand( () -> {
          if (lastColor == 0.69) {
            turnYellow().schedule();
          } else if (lastColor == 0.91) {
            turnPurple().schedule();
          } else {
            idle().schedule();
          }
        })
      )
    );
  }

  public Command idle () {
    return new FunctionalCommand(
      () -> {},
      () -> command = 0.01,
      (interrupted) -> {},
      () -> true,
      (Subsystem) this
      );
  }
  
  @Override
  public void periodic() {
    LEDsOutput.set(command);
    Telemetry.setValue("LEDS/Blinkin/value", LEDsOutput.get());
  }

  @Override
  public void simulationPeriodic() {}
}