package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.LL;;

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

  public double getyaw() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  }
  

  public double getpitch() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  }

  public double getarea() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
  }

  public double getXDistance() {
    return LL.SLOPE * getarea() + LL.YINT;
  }

  public double getYDistance() {
    return Math.tan(getyaw())* getXDistance();
  }


}

// <----- Discussion Thread Here!!!!!