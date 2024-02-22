package frc.robot.commands.Swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain.Drivetrain;

public class PathToPoint extends SequentialCommandGroup {
    private Drivetrain drivetrain;
    private Pose2d targetPose;

    public enum WayPoints {
        kAmp,
        kHumanPlayer,
        kSpeakerLeft,
        kSpeakerRight,
        kSpeakerCenter
    }

    /**
     * Command to set the drivetrain to a specific position on the field while
     * avoiding field obstacles
     */
    public PathToPoint() {
        this(new Pose2d(8.27, 2.42, Rotation2d.fromDegrees(0)));
    }

    // TODO (@Hellothere212): display the trajectory onto smartdashboard field using
    // Robot.mField

    // How to implement

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
                AutoBuilder.pathfindToPoseFlipped(
                        this.targetPose,
                        new PathConstraints(
                                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared,
                                Constants.AutoConstants.kMaxAngularSpeedMetersPerSecond,
                                Constants.AutoConstants.kMaxAngularAccelerationMetersPerSecondSquared),
                        0.0,
                        0.0));
    }

    /**
     * @deprecated Kind of useless now that we have an {@link AutoBuilder#pathfindToPoseFlipped(Pose2d, PathConstraints, double)} constructor available
     * <p>
     * Command to set the drivetrain to a specific position on the field while
     * avoiding field obstacles
     * 
     * @param wayPoint the position that the robot should move to
     */
    public PathToPoint(WayPoints wayPoint) {
        drivetrain = Drivetrain.getInstance();
        boolean isRed = Robot.isRed();

        switch (wayPoint) {
            case kAmp:
                if (isRed) {
                    targetPose = Constants.AutoConstants.WayPoints.Red.kAmp;
                } else {
                    targetPose = Constants.AutoConstants.WayPoints.Blue.kAmp;
                }
                break;
            case kHumanPlayer:
                if (isRed) {
                    targetPose = Constants.AutoConstants.WayPoints.Red.kHumanPlayer;
                } else {
                    targetPose = Constants.AutoConstants.WayPoints.Blue.kHumanPlayer;
                }
                break;
            case kSpeakerLeft:
                if (isRed) {
                    targetPose = Constants.AutoConstants.WayPoints.Red.kSpeakerLeft;
                } else {
                    targetPose = Constants.AutoConstants.WayPoints.Blue.kSpeakerLeft;
                }
                break;
            case kSpeakerRight:
                if (isRed) {
                    targetPose = Constants.AutoConstants.WayPoints.Red.kSpeakerRight;
                } else {
                    targetPose = Constants.AutoConstants.WayPoints.Blue.kSpeakerRight;
                }
                break;
            case kSpeakerCenter:
                if (isRed) {
                    targetPose = Constants.AutoConstants.WayPoints.Red.kSpeakerCenter;
                } else {
                    targetPose = Constants.AutoConstants.WayPoints.Blue.kSpeakerCenter;
                }
                break;
            default:
                targetPose = drivetrain.getPose();
                break;
        }

        this.setName("PathToPoint");
        this.addRequirements(drivetrain);

        this.addCommands(
                AutoBuilder.pathfindToPose(
                        targetPose,
                        new PathConstraints(
                                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared,
                                Constants.AutoConstants.kMaxAngularSpeedMetersPerSecond,
                                Constants.AutoConstants.kMaxAngularAccelerationMetersPerSecondSquared),
                        0.0,
                        0.0));
    }
}
