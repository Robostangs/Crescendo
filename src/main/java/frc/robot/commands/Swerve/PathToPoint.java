package frc.robot.commands.Swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain.Drivetrain;

public class PathToPoint extends SequentialCommandGroup {
        private Drivetrain drivetrain;
        private Pose2d targetPose;

        /**
         * Uses {@code AutoBuilder} to drive to {@code targetPose} 
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
                                                                Constants.AutoConstants.AutoSpeeds.kMaxSpeedMetersPerSecond,
                                                                Constants.AutoConstants.AutoSpeeds.kMaxAccelerationMetersPerSecondSquared,
                                                                Constants.AutoConstants.AutoSpeeds.kMaxAngularSpeedRadiansPerSecond,
                                                                Constants.AutoConstants.AutoSpeeds.kMaxAngularAccelerationRadiansPerSecondSquared),
                                                0.0,
                                                0.0));
        }
}
