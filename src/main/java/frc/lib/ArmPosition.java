package frc.lib;

public class ArmPosition {
    private double stage1Angle = 0;
    private double stage2Angle = 0;
    private double stage3Angle = 0;

    public ArmPosition (double stage1Angle, double stage2Angle, double stage3Angle) {
        this.stage1Angle = stage1Angle;
        this.stage2Angle = stage2Angle;
        this.stage3Angle = stage3Angle;
    }

    public double getStage1Angle () {
        return stage1Angle;
    }

    public double getStage2Angle () {
        return stage2Angle;
    }

    public double getStage3Angle () {
        return stage3Angle;
    }
}
