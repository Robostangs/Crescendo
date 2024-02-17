package frc.robot.commands.Swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain.Drivetrain;

public class PathToPoint extends SequentialCommandGroup {
    private Drivetrain drivetrain;
    private Pose2d targetPose;

    /**
     * Command to set the drivetrain to a specific position on the field while
     * avoiding field obstacles
     */
    public PathToPoint() {
        this(new Pose2d(8.27, 2.42, Rotation2d.fromDegrees(0)));
    }

    // TODO: display the trajectory onto smartdashboard field using Robot.mField;

    // How to implement
    // xDrive.a().onTrue(new PathToPoint());

    /**
     * Command to set the drivetrain to a specific position on the field while
     * avoiding field obstacles
     * 
     * @param targetPose the position that the robot should move to
     */
    public PathToPoint(Pose2d targetPose) {
        drivetrain = Drivetrain.getInstance();

        this.targetPose = targetPose;

        this.setName("PathToPoint");
        this.addRequirements(drivetrain);

        this.addCommands(
                AutoBuilder.pathfindToPose(
                        this.targetPose,
                        new PathConstraints(
                                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared,
                                Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond,
                                Constants.AutoConstants.kMaxAngularAccelerationRadiansPerSecondPerSecond),
                        0.0, // Goal end velocity in meters/sec
                        0.0 // Rotation delay distance in meters. This is how far the robot should travel
                            // before attempting to rotate.
                ));
    }

    // public Command followthePath(Pose2d startPose) {
    // drivetrain.applyRequest(() -> drive.withRotationalDeadband(
    // drivetrain.getPose().getRotation().getDegrees() -
    // drivetrain.getPose().getRotation().getDegrees()));

    // PathConstraints constraints = new PathConstraints(5, 6, 540d, 720d);

    // Pose2d targetPose = new Pose2d(8.27, 2.42, Rotation2d.fromDegrees(0));

    // Command pathfindingCommand = AutoBuilder.pathfindToPose(
    // targetPose,
    // constraints,
    // 0.0, // Goal end velocity in meters/sec
    // 0.0 // Rotation delay distance in meters. This is how far the robot should
    // travel
    // // before attempting to rotate.
    // );
    // return pathfindingCommand;

    // }
}
