package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class HolonomicController {
    private ProfiledPIDController xController;
    private ProfiledPIDController yController;
    private ProfiledPIDController thetaController;

    public HolonomicController(
        ProfiledPIDController xController, ProfiledPIDController yController, ProfiledPIDController thetaController) {
        thetaController.enableContinuousInput( -Math.PI, Math.PI );

        this.xController = xController;
        this.yController = yController;
        this.thetaController = thetaController;

        setTolerance( new Pose2d() );
    }

    public void reset(Pose2d startPose) {
        reset( startPose, new ChassisSpeeds() );
    }

    public void reset(Pose2d startPose, ChassisSpeeds targetChassisSpeeds) {
        xController.reset( startPose.getX() );
        yController.reset( startPose.getY() );
        thetaController.reset( startPose.getRotation().getRadians() );
    }

    public boolean atGoal() {
        return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
    }

    public Pose2d getPositionGoal() {
        return new Pose2d(
            new Translation2d(
                xController.getGoal().position, 
                yController.getGoal().position), 
            new Rotation2d(
                thetaController.getGoal().position
            ));
    }

    public Pose2d getPositionSetpoint() {
        return new Pose2d(
            new Translation2d(
                xController.getSetpoint().position, 
                yController.getSetpoint().position ), 
            new Rotation2d(
                thetaController.getSetpoint().position ) );
    }

    public ChassisSpeeds getVelocityGoal() {
        return new ChassisSpeeds(
            xController.getGoal().velocity,
            yController.getGoal().velocity,
            thetaController.getGoal().velocity );
    }

    public ChassisSpeeds getVelocitySetpoint() {
        return new ChassisSpeeds(
            xController.getSetpoint().velocity,
            yController.getSetpoint().velocity,
            thetaController.getSetpoint().velocity );
    }

    public ChassisSpeeds calculate(Pose2d goalPose, Pose2d currentPose) {
        return calculate(goalPose, new ChassisSpeeds(), currentPose);
    }

    public ChassisSpeeds calculate(Pose2d goalPose, ChassisSpeeds goalSpeed, Pose2d currentPose) {
        return new ChassisSpeeds(
            xController.calculate( 
                currentPose.getX(), 
                new TrapezoidProfile.State(
                    goalPose.getX(),
                    goalSpeed.vxMetersPerSecond) ),

            yController.calculate( 
                currentPose.getY(), 
                new TrapezoidProfile.State(
                    goalPose.getY(),
                    goalSpeed.vyMetersPerSecond) ),

            thetaController.calculate( 
                currentPose.getRotation().getDegrees(), 
                new TrapezoidProfile.State(
                    goalPose.getRotation().getDegrees(),
                    goalSpeed.vxMetersPerSecond) ) );
    }

    public ChassisSpeeds calculate(Pose2d robotPose) {
        return new ChassisSpeeds(
            xController.calculate( robotPose.getX() ),
            yController.calculate( robotPose.getY() ),
            thetaController.calculate( robotPose.getRotation().getRadians() )
        );
    }

    public ChassisSpeeds calculateWithFF(Pose2d goalPose, Pose2d currentPose) {
        return calculateWithFF(goalPose, new ChassisSpeeds(), currentPose);
    }

    public ChassisSpeeds calculateWithFF(Pose2d goalPose, ChassisSpeeds goalSpeed, Pose2d currentPose) {
        return new ChassisSpeeds(
            xController.calculate( 
                currentPose.getX(), 
                new TrapezoidProfile.State(
                    goalPose.getX(),
                    goalSpeed.vxMetersPerSecond) )
            + xController.getSetpoint().velocity,

            yController.calculate( 
                currentPose.getY(), 
                new TrapezoidProfile.State(
                    goalPose.getY(),
                    goalSpeed.vyMetersPerSecond) )
            + yController.getSetpoint().velocity,

            thetaController.calculate( 
                currentPose.getRotation().getDegrees(), 
                new TrapezoidProfile.State(
                    goalPose.getRotation().getDegrees(),
                    goalSpeed.omegaRadiansPerSecond) )
            + thetaController.getSetpoint().velocity
            );
    }

    public ChassisSpeeds calculateWithFF(Pose2d robotPose) {
        return new ChassisSpeeds(
            xController.calculate( robotPose.getX() )
            + xController.getSetpoint().velocity,
            yController.calculate( robotPose.getY() )
            + yController.getSetpoint().velocity,
            thetaController.calculate( robotPose.getRotation().getRadians()
            + thetaController.getSetpoint().velocity ) );
    }

    public void setGoal(Pose2d goalPose) {
        setGoal(goalPose, new ChassisSpeeds());
    }

    public void setGoal(Pose2d goalPose, ChassisSpeeds goalSpeed) {
        xController.setGoal( 
            new TrapezoidProfile.State(
                goalPose.getX(),
                goalSpeed.vxMetersPerSecond) );
        yController.setGoal( 
            new TrapezoidProfile.State(
                goalPose.getY(),
                goalSpeed.vyMetersPerSecond) );
        thetaController.setGoal( 
            new TrapezoidProfile.State(
                goalPose.getRotation().getRadians(),
                goalSpeed.omegaRadiansPerSecond) );
    }

    public void setTolerance(Pose2d tolerance) {
        xController.setTolerance( tolerance.getX() );
        yController.setTolerance( tolerance.getY() );
        thetaController.setTolerance( tolerance.getRotation().getRadians() );
    }

    public void xControllerIRange(double range) {
        xControllerIRange( -range, range );
    }

    public void yControllerIRange(double range) {
        yControllerIRange( -range, range );
    }

    public void thetaControllerIRange(double range) {
        thetaControllerIRange( -range, range );
    }

    public void xControllerIRange(double lowerBound, double higherBound) {
        xController.setIntegratorRange( lowerBound, higherBound );
    }

    public void yControllerIRange(double lowerBound, double higherBound) {
        yController.setIntegratorRange( lowerBound, higherBound );
    }

    public void thetaControllerIRange(double lowerBound, double higherBound) {
        thetaController.setIntegratorRange( lowerBound, higherBound );
    }
}