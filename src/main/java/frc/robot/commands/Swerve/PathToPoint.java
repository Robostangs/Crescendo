package frc.robot.commands.Swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.subsystems.Drivetrain.SwerveRequest;
import frc.robot.subsystems.Drivetrain.SwerveModule.DriveRequestType;

public class PathToPoint {
    private static Drivetrain drivetrain = Drivetrain.getInstance();
    private static SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(Constants.SwerveConstants.kSpeedAt12VoltsMetersPerSecond * 0.08)
    .withRotationalDeadband(0)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 
    public static Command followthePath(Pose2d startPose) {
        drivetrain.applyRequest(() -> drive.withRotationalDeadband(drivetrain.getPose().getRotation().getDegrees() - drivetrain.getPose().getRotation().getDegrees()));

        PathConstraints constraints = new PathConstraints(5, 6, 540d, 720d);

  
        Pose2d targetPose = new Pose2d(8.27, 2.42, Rotation2d.fromDegrees(0));

        Command pathfindingCommand = AutoBuilder.pathfindToPose(
        targetPose,
        constraints,
        0.0, // Goal end velocity in meters/sec
        0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
);
        return pathfindingCommand;
      

    }
}
