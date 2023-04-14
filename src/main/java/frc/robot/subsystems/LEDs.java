package frc.robot.subsystems;
import java.util.Arrays;

import com.ctre.phoenix.led.ColorFlowAnimation;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class LEDs extends SubsystemBase {
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private int m_r = 200;
  private int m_g = 233;
  private int m_b = 233;
  private int m_r2 = 0;
  private int m_g2 = 0;
  private int m_b2 = 0;
  private double[] color = {0, 0, 0};
  private int leader = 0;
  private int trailLength = 4;
  private int[] trail = {1, 2, 3, 4};

  public LEDs () {

    m_led = new AddressableLED(1);
   
    m_ledBuffer = new AddressableLEDBuffer(18);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void setColor ( int r, int g, int b) {
    m_r = r;
    m_g = g;
    m_b = b;
  }

  public Command turnPurple () { return new InstantCommand( () -> {
    setColor((int)Math.floor(Color.kPurple.red*255), (int)Math.floor(Color.kPurple.green*255), (int)Math.floor(Color.kPurple.blue*255));
  }); }
  public Command turnYellow () { return new InstantCommand( () -> {
    setColor((int)Math.floor(Color.kOrange.red*255), (int)Math.floor(Color.kOrange.green*255), (int)Math.floor(Color.kOrange.blue*255));
  } ); }
  public Command flashRed () { return new InstantCommand( () -> {
    m_r2 = m_r;
    m_g2 = m_g;
    m_b2 = m_b;
    setColor(255, 0, 0);
  } ).andThen(new WaitCommand(0.5)).andThen(new InstantCommand( () -> {
    setColor(m_r2, m_g2, m_b2);
  } )); }
  public Command flashGreen () { return new InstantCommand( () -> {
    m_r2 = m_r;
    m_g2 = m_g;
    m_b2 = m_b;
    setColor(0, 255, 0);
  } ).andThen(new WaitCommand(0.5)).andThen(new InstantCommand( () -> {
    setColor(m_r2, m_g2, m_b2);
  } )); }
  
  @Override
  public void periodic() {
    // TODO iterate leader and trail to advance the sequence

    color = RGBtoHSV(m_r, m_g, m_b);
    int temp = 0;

    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      if (i == leader) {
        m_ledBuffer.setHSV(i, (int) color[0], (int) color[1], 1);
      } else {
        temp = Arrays.stream(trail).boxed().toList().indexOf(i);
        m_ledBuffer.setHSV(i, (int) color[0], (int) color[1], (temp == -1 ? 0 : (int) ((trailLength - temp)/trailLength)));
      }
    }

    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  @Override
  public void simulationPeriodic() {}

  public static double[] RGBtoHSV(double r, double g, double b){

    double h, s, v;

    double min, max, delta;

    min = Math.min(Math.min(r, g), b);
    max = Math.max(Math.max(r, g), b);

    // V
    v = max;

     delta = max - min;

    // S
     if( max != 0 )
        s = delta / max;
     else {
        s = 0;
        h = -1;
        return new double[]{h,s,v};
     }

    // H
     if( r == max )
        h = ( g - b ) / delta; // between yellow & magenta
     else if( g == max )
        h = 2 + ( b - r ) / delta; // between cyan & yellow
     else
        h = 4 + ( r - g ) / delta; // between magenta & cyan

     h *= 60;    // degrees

    if( h < 0 )
        h += 360;

    return new double[]{h,s,v};
  }
}