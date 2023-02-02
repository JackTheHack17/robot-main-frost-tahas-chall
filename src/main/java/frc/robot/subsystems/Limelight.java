package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.LL;
import frc.robot.Constants.DRIVETRAIN;


// Test code to change between pipelines on limelight
public class Limelight extends InstantCommand {
  private NetworkTable limelight;
  private boolean pipelineIndex;

  public Limelight(){
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
    pipelineIndex = false;
  }

  public void switchPipeline() {
    limelight.getEntry("pipeline").setNumber(!pipelineIndex? 1 : 0);
  }

  public int getPipeLineIndex() {
    return pipelineIndex? 1 : 0;
  }
  // You wanna check this out?
  // https://github.com/STMARobotics/swerve-test
  // Actually working swerve code to stay alligned to apriltag
  // Well A and B are used, Y for switching pipelines, You got options: X, Right Bumper, or left bumboer
  // Which one you wanna do?
  // Ok, you wanna do that? im sure bot pose requires math that im not willing to do, cody you can do the math... 
  // ok we have to set up bot pose from limelight apis
  public double getyaw() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  }
  

  public double getPitch() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  }

  public double getArea() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
  }

  public double getXDistance() {
    return LL.SLOPE * getArea() + LL.YINT;
  }

  public double getYDistance() {
    return Math.tan(getyaw())* getXDistance();
  }
  /**
   * Align with a limelight target
   */
  public void alignTarget(Drivetrain Drive)
  {
    double L_X = (Math.tan(getPitch()) * getXDistance()) - (DRIVETRAIN.ROBOT_WIDTH/2);
    double L_Y = 0.0;
    double R_X = 0.0;
    Drive.joystickDrive(L_X,L_Y,R_X);
    //Comments Here: 
  }

}

// <----- Discussion Thread Here!!!!!