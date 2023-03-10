package frc.robot.subsystems;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
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
 
  AddressableLED m_leds = new AddressableLED(0);
  AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(500);
  int m_rainbowFirstPixelHue;
  double lastChange;
  boolean on = true;



  public LEDs () {
   
    m_leds.setLength(m_ledBuffer.getLength());

    // Set the data
    m_leds.setData(m_ledBuffer);
    m_leds.start();
  }

  public void rainbow() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }

  public void flashGreen(){
    double timestamp = Timer.getFPGATimestamp();
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
		if (timestamp - lastChange > 0.5){
			on = !on;
			lastChange = timestamp;
		}
    
		if (on){
			m_ledBuffer.setRGB(i, 0, 255, 0);
		} else {
			m_ledBuffer.setRGB(i, 0, 0, 0);
		}
  }
  }

  public void flashRed(){
    double timestamp = Timer.getFPGATimestamp();
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
		if (timestamp - lastChange > 0.5){
			on = !on;
			lastChange = timestamp;
		}
    
		if (on){
			m_ledBuffer.setRGB(i, 255, 0, 0);
		} else {
			m_ledBuffer.setRGB(i, 0, 0, 0);
		}
  }
  }


  public void flashCube(){
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the HSV values for red
      m_ledBuffer.setHSV(i, 60, 255, 255);
   }
  }
  
  @Override
  public void periodic() {
   // set(r, g, b);
//    Telemetry.setValue("m_leds/Blinkin/value", m_ledBuffer.get());
  }

  @Override
  public void simulationPeriodic() {}
}