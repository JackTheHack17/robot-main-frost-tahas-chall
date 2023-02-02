package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.LL;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

// Test code to change between pipelines on limelight
public class Limelight extends InstantCommand {
  private NetworkTable limelight;
  private boolean pipelineIndex;
  private double[] posevalues;

  public Limelight() {
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
    pipelineIndex = false;
  }

  public void switchPipeline() {
    limelight.getEntry("pipeline").setNumber(!pipelineIndex ? 1 : 0);
  }

  public int getPipeLineIndex() {
    return pipelineIndex ? 1 : 0;
  }

  // You wanna check this out?
  // https://github.com/STMARobotics/swerve-test
  public double getyaw() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  }

  public double getPitch() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  }

  public double getArea() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
  }

  public Pose2d getPoseValues(String team) {
    if(team == "red") {
      posevalues = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    }
    if(team == "red") {
      posevalues = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpired").getDoubleArray(new double[6]);
    }
    Translation2d translate = new Translation2d(posevalues[0], posevalues[1]);
    Rotation2d rotation = new Rotation2d(posevalues[3], posevalues[4]);
    return new Pose2d(translate, rotation);
  }

  public double getTapeXDistance() {
    return LL.SLOPE * getArea() + LL.YINT;
  }

  public double getTapeYDistance() {
    return Math.tan(getyaw()) * getTapeXDistance();
  }

  /**
   * Align with a limelight target
   */
  public double alignTapeTarget(Drivetrain Drive) {
    PIDController LLAlign = new PIDController(0, 0, 0);
    return LLAlign.calculate(getTapeXDistance(), 0);
  }

}

// <----- Discussion Thread Here!!!!!