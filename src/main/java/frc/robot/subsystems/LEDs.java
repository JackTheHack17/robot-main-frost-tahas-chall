package frc.robot.subsystems;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//import frc.lib.Telemetry;
import frc.robot.Constants.*;

// https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf

public class LEDs extends SubsystemBase {
//  PWMSparkMax LEDsOutput = new PWMSparkMax(Constants.PWM.BLINKIN_ID);
  int r;
  int g;
  int b;
  int lastr = 0;
  int lastg = 0;
  int lastb = 0;
  AddressableLED LEDS = new AddressableLED(0);
  AddressableLEDBuffer LEDsOutput = new AddressableLEDBuffer(240);

  public LEDs () {
    r = 0;
    g = 0;
    b = 0;
  }

  public Command turnYellow () {
    return new InstantCommand( () -> {r = LED.YELLOWR; g = LED.YELLOWG; b = LED.YELLOWB; 
                                    lastr = LED.YELLOWR; lastg = LED.YELLOWG; lastb = LED.YELLOWB;} );
  }

  public Command turnPurple () {
    return new InstantCommand( () -> {r = LED.PURPLER; g = LED.PURPLEG; b = LED.PURPLEB; 
                                    lastr = LED.PURPLER; lastg = LED.PURPLEB; lastb = LED.PURPLEB;} );
  }

  public Command flashRed () {
    return new ParallelCommandGroup(
      new InstantCommand( () -> {r = LED.REDR; g = LED.REDG; b = LED.REDB;} ), 
      new SequentialCommandGroup(
        new WaitCommand(1), 
        new InstantCommand( () -> {
          if (lastr == LED.YELLOWR) {
            turnYellow().schedule();
          } else if (lastr == LED.PURPLER) {
            turnPurple().schedule();
          } else {
            idle().schedule();
          }
        })
      )
    );
  }

  public Command flashGreen () {
    return new ParallelCommandGroup(
      new InstantCommand( () -> {r = LED.GREENR; g = LED.GREENB; b = LED.GREENB;} ), 
      new SequentialCommandGroup(
        new WaitCommand(1), 
        new InstantCommand( () -> {
          if (lastr == LED.YELLOWR) {
            turnYellow().schedule();
          } else if (lastr == LED.PURPLER) {
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
      () -> {r = 0; b = 0; g = 0;},
      (interrupted) -> {},
      () -> true,
      (Subsystem) this
      );
  }

  public void set(int r, int g, int b) {
    for (var i = 0; i < LEDsOutput.getLength(); i++) {
      LEDsOutput.setRGB(i, r, g, b);
    }

  }
  
  @Override
  public void periodic() {
    set(r, g, b);
//    Telemetry.setValue("LEDS/Blinkin/value", LEDsOutput.get());
  }

  @Override
  public void simulationPeriodic() {}
}