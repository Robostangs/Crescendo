package frc.robot.commands.AutoCommands;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Alert;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Alert.AlertType;
import frc.robot.subsystems.Drivetrain.Drivetrain;

import java.util.List;
import java.util.ArrayList;

public class PathPlannerCommand extends SequentialCommandGroup {
    Drivetrain drivetrain = Drivetrain.getInstance();

    // static Supplier<Pose2d> m_poseSupplier = () -> Drivetrain.getInstance().getPose();
    // static Supplier<ChassisSpeeds> m_speedSupplier = () -> Drivetrain.getInstance().getCurrentRobotChassisSpeeds();
    // static Consumer<ChassisSpeeds> m_speedConsumer = speeds -> Drivetrain.getInstance()
    //         .setControl(Drivetrain.getInstance().autoRequest.withSpeeds(speeds));
    // static HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(
    //         Constants.SwerveConstants.maxModuleSpeed, Constants.SwerveConstants.driveBaseRadius,
    //         new ReplanningConfig());
    // static BooleanSupplier shouldFlipPathSupplier = Robot::isRed;

    // public PathPlannerCommand() {
    //     Pose2d startingPose;

    //     switch (Robot.startingPose.getSelected()) {
    //         case "amp":
    //             startingPose = Constants.AutoConstants.WayPoints.Blue.AmpStartPosition;
    //             break;
    //         case "center":
    //             startingPose = Constants.AutoConstants.WayPoints.Blue.CenterStartPosition;
    //             break;
    //         case "stage":
    //             startingPose = Constants.AutoConstants.WayPoints.Blue.StageStartPosition;
    //             break;
    //         default:
    //             startingPose = drivetrain.getPose();
    //             break;
    //     }

    //     this.addCommands(new InstantCommand(() -> drivetrain.seedFieldRelative(startingPose)));

    //     // this takes care of first note

    //     // this is for far pieces being the first note
    //     if (Robot.firstNote.getSelected().contains("far")) {
    //         this.addCommands(getPathCommand(Robot.startingPose.getSelected() + " " + "far" + " " + "leave"),
    //         new DriveToNote().raceWith(new DeployAndIntake(true)),
            
    //         );
    //     }

    //     else {
    //         this.addCommands(getPathCommand(
    //                 Robot.startingPose.getSelected() + " " + "start" + " " + "to" + " " + Robot.firstNote.getSelected())
    //                 .raceWith(new DeployAndIntake(true)),
    //                 ShootCommandFactory.getAutonomousShootCommand());
    //     }

    //     this.addCommands(getPathCommand(Robot.startingPose.getSelected() + " start"));

    //     // PathPlannerAuto.getPathGroupFromAutoFile(lastAutoName)
    //     PathPlannerPath.fromPathFile("test");
    // }

    // public static FollowPathHolonomic getPathCommand(String path) {
    //     return new FollowPathHolonomic(PathPlannerPath.fromPathFile("path"), m_poseSupplier, m_speedSupplier,
    //             m_speedConsumer, config, shouldFlipPathSupplier, Drivetrain.getInstance());
    // }

    private static String lastAutoName;
    private static Alert nullAuto = new Alert("Null auto", AlertType.WARNING);
    private static Alert publishfail = new Alert("Publishing failed", AlertType.ERROR);
    private static Alert noAutoSelected = new Alert("No Auto Selected", AlertType.WARNING);

    public static void publishTrajectory(String autoName) {
        if (autoName == null) {
            Robot.autoField.getObject(Constants.AutoConstants.kFieldObjectName)
                    .setPose(new Pose2d(-5, -5, Rotation2d.fromDegrees(0)));
            Robot.teleopField.getObject(Constants.AutoConstants.kFieldObjectName)
                    .setPose(new Pose2d(-5, -5, Rotation2d.fromDegrees(0)));
            lastAutoName = "null";
            noAutoSelected.set(true);
            nullAuto.set(false);
            return;
        }

        else if (autoName.equals(lastAutoName)) {
            return;
        }

        else {
            lastAutoName = autoName;
        }

        List<Pose2d> poses = new ArrayList<>();
        poses.clear();

        try {
            PathPlannerAuto.getPathGroupFromAutoFile(autoName).forEach((path) -> path.getAllPathPoints()
                    .forEach((point) -> {
                        Pose2d pose = new Pose2d(point.position, point.position.getAngle());
                        if (Robot.isRed()) {
                            pose = GeometryUtil.flipFieldPose(pose);
                        }

                        poses.add(pose);
                    }));
            if (Robot.isRed()) {
                Robot.teleopField.getObject("Starting Pose")
                        .setPose(GeometryUtil.flipFieldPose(PathPlannerAuto.getStaringPoseFromAutoFile(autoName)));
            }

            else {
                Robot.teleopField.getObject("Starting Pose")
                        .setPose(PathPlannerAuto.getStaringPoseFromAutoFile(autoName));
            }

            nullAuto.set(false);
            publishfail.set(false);
            noAutoSelected.set(false);
        }

        catch (RuntimeException e) {
            System.out.println("Null Auto: " + autoName);
            nullAuto.setText("Null auto: " + autoName);
            nullAuto.set(true);
        }

        catch (Exception e) {
            publishfail.set(true);
            e.printStackTrace();
        }

        Robot.autoField.getObject(Constants.AutoConstants.kFieldObjectName).setPoses(poses);
        Robot.teleopField.getObject(Constants.AutoConstants.kFieldObjectName).setPoses(poses);
    }

    public static void unpublishTrajectory() {
        publishTrajectory(null);
    }
}