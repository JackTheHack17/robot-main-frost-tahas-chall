package frc.lib;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import java.util.function.Supplier;

public class moveToPosition {
    private ProfiledPIDController xController;
    private ProfiledPIDController yController;
    private ProfiledPIDController thetaController;
    private Supplier<Pose2d> currentPose;
    private Supplier<ChassisSpeeds> currentChassisSpeeds;
    private Drivetrain driveSub;

    public moveToPosition(ProfiledPIDController xController, 
                          ProfiledPIDController yController, 
                          ProfiledPIDController thetaController,
                          Drivetrain driveSub) {
        this.xController = xController;
        this.yController = yController;
        this.thetaController = thetaController;
        this.driveSub = driveSub;
        currentPose = () -> driveSub.getPose();
        currentChassisSpeeds = () -> driveSub.getChassisSpeeds();

        this.thetaController.enableContinuousInput(-180, 180);
    }

    public Command generateMoveToPositionCommandTerminating( Pose2d startPose, Pose2d targetPose, Pose2d tolerance ) {
        return generateMoveToPositionCommand( startPose, targetPose, new ChassisSpeeds(), tolerance )
            .until(() -> xController.atGoal() && yController.atGoal());
    }

    public Command generateMoveToPositionCommand( Pose2d startPose,Pose2d targetPose, Pose2d tolerance ) {
        return generateMoveToPositionCommand( startPose, targetPose, new ChassisSpeeds(), tolerance );
    }

    public Command generateMoveToPositionCommandTerminating( Pose2d startPose, Pose2d targetPose, ChassisSpeeds targetChassisSpeeds, Pose2d tolerance ) {
        return generateMoveToPositionCommand( startPose, targetPose, targetChassisSpeeds, tolerance)
                .until(() -> xController.atGoal() && yController.atGoal()); 
    }

    public Command generateMoveToPositionCommand( Pose2d startPose, final Pose2d targetPose, ChassisSpeeds targetChassisSpeeds, Pose2d tolerance ) {
        final double x = targetPose.getX();
        final double y = targetPose.getY();
        final double rotation = targetPose.getRotation().getDegrees();

        return new FunctionalCommand(
            () -> {
                setTolerance(tolerance);

                System.out.println(startPose.getX());
                System.out.println(startPose.getY());
                System.out.println(startPose.getRotation().getDegrees());

                System.out.println(x);
                System.out.println(y);
                System.out.println(rotation);

                xController.reset( startPose.getX() );
                yController.reset( startPose.getY() );
                thetaController.reset( startPose.getRotation().getDegrees() );

                xController.setGoal(x);
                yController.setGoal(y);
                thetaController.setGoal(rotation);
            }, 
            () -> {
                double xPID = xController.calculate( currentPose.get().getX() );
                double yPID = yController.calculate( currentPose.get().getY() );
                double thetaPID = thetaController.calculate( currentPose.get().getRotation().getDegrees());

                double xFF = xController.getSetpoint().velocity;
                double yFF = yController.getSetpoint().velocity;
                double thetaFF = 0;//thetaController.getSetpoint().velocity;

                driveSub.field2d.getObject("Setpoint").setPose(
                    new Pose2d(
                        xController.getGoal().position,
                        yController.getGoal().position,
                        Rotation2d.fromDegrees( thetaController.getGoal().position ) ) );

                driveSub.driveFromModuleStates(
                    driveSub.getKinematics().toSwerveModuleStates(
                        new ChassisSpeeds( 
                            xPID + xFF,
                            yPID + yFF,
                            thetaPID + thetaFF ) ) );
            }, 
            (interrupted) -> { driveSub.joystickDrive(0, 0, 0); }, 
            () -> false,
            driveSub) ;
    }

    public void setTolerance(Pose2d pose) {
        xController.setTolerance(pose.getX());
        yController.setTolerance(pose.getY());
        thetaController.setTolerance(pose.getRotation().getDegrees());
    }
}