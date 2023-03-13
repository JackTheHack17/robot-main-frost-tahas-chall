package frc.lib;

import static frc.robot.Constants.ARM.STAGE_1_LENGTH;
import static frc.robot.Constants.ARM.STAGE_2_LENGTH;

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

    public double getXPosition() {
        return STAGE_1_LENGTH * Math.cos(Math.toRadians(stage1Angle)) + STAGE_2_LENGTH * Math.cos(Math.toRadians(stage2Angle));
    }

    public double getYPosition() {
        return -(STAGE_1_LENGTH * Math.sin(Math.toRadians(stage1Angle)) + STAGE_2_LENGTH * Math.sin(Math.toRadians(stage2Angle)));
    }
}
