package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Limelight;

public class VisionSubsystem extends SubsystemBase{
    private Limelight centerLimelight;

    public VisionSubsystem() {
        centerLimelight = new Limelight("limelight-limeone");
    }

    public Limelight getCenterLimelight() {
        return centerLimelight;
    }

    @Override
    public void periodic() {
        centerLimelight.periodic();
    }
}
