package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.lib.Telemetry;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.Supplier;

public class moveToPosition {
    private Supplier<Pose2d> currentPose;
    private Supplier<ChassisSpeeds> currentChassisSpeeds;
    private Drivetrain driveSub;
    private Pose2d target = new Pose2d();

    public moveToPosition( Drivetrain driveSub ) {
        this.driveSub = driveSub;
        currentPose = () -> driveSub.getPose();
        currentChassisSpeeds = () -> driveSub.getChassisSpeeds();
    }

    public Command generateMoveToPositionCommand( 
        Pose2d targetPose, Pose2d tolerance, HolonomicController controller ) {
        return generateMoveToPositionCommand( 
            targetPose, 
            new ChassisSpeeds(), 
            tolerance, 
            controller );
    }

    public Command generateMoveToPositionCommand( 
        Pose2d targetPose, ChassisSpeeds targetChassisSpeeds, 
        Pose2d tolerance, HolonomicController controller ) {
        return new FunctionalCommand(
            () -> {
                this.target = targetPose;
                controller.setTolerance( tolerance );
                controller.reset( currentPose.get(), currentChassisSpeeds.get() );
                controller.setGoal(targetPose, targetChassisSpeeds);
            },
            () -> {
                driveSub.driveFromChassisSpeeds(
                    discretize( 
                        controller.calculate( currentPose.get() ) ) );

                driveSub.field2d.getObject( "Setpoint" ).setPose( controller.getPositionSetpoint() );
                driveSub.field2d.getObject( "Goal" ).setPose( controller.getPositionGoal() );
                Telemetry.getValue("PathPlanner/AtGoal", controller.atGoal() );
            }, 
            (interrupted) -> { driveSub.joystickDrive(0, 0, 0); }, 
            () -> controller.atGoal(),
            driveSub) ;
    }

    public ChassisSpeeds discretize(ChassisSpeeds speeds) {
        double dt = 0.02;
        var desiredDeltaPose = new Pose2d(
          speeds.vxMetersPerSecond * dt, 
          speeds.vyMetersPerSecond * dt, 
          new Rotation2d(speeds.omegaRadiansPerSecond * dt * 3)
        );
        var twist = new Pose2d().log(desiredDeltaPose);
    
        return new ChassisSpeeds((twist.dx / dt), (twist.dy / dt), (speeds.omegaRadiansPerSecond));
    }

    public Pose2d getTarget() {
        return target;
    }
}