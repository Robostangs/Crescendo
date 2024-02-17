package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.feeder.DeployAndIntake;
import frc.robot.commands.shooter.AimAndShoot;
import frc.robot.commands.shooter.FeedAndShoot;
import frc.robot.commands.shooter.SetPoint;
import frc.robot.subsystems.Drivetrain.Drivetrain;

import java.util.List;
import java.util.ArrayList;

public class PathPlannerCommand extends SequentialCommandGroup {
    private Drivetrain swerve;
    private Command auto;
    private Pose2d startPose;

    private static String lastAutoName;

    public PathPlannerCommand(String autoName, boolean shoot) {
        NamedCommands.registerCommand("Intake", new DeployAndIntake(true).withTimeout(0.5));
        NamedCommands.registerCommand("AimAndShoot", new AimAndShoot().withTimeout(1));
        // NamedCommands.registerCommand("Shoot", new PrintCommand("Shoot Command,
        // PathPlanner"));
        // SetPoint(Constants.ArmConstants.SetPoints.kSpeakerClosestPoint).withTimeout(0.1));
        NamedCommands.registerCommand("Stow", new SetPoint(Constants.ArmConstants.SetPoints.kIntake).withTimeout(0.3));

        swerve = Drivetrain.getInstance();

        this.addCommands(
                AutoBuilder.followPath(
                        PathPlannerPath.fromPathFile("left start 1 piece")));

        // try {
        // startPose = new
        // Pose2d(PathPlannerAuto.getPathGroupFromAutoFile(autoName).get(0)
        // .getPreviewStartingHolonomicPose().getTranslation(),
        // Rotation2d.fromDegrees(0));
        // auto = AutoBuilder.buildAuto(autoName);
        // } catch (Exception e) {
        // this.setName("Null Auto");
        // System.out.println("Null auto");
        // auto = new PrintCommand("Null Auto");
        // startPose = new Pose2d(2, 4, Rotation2d.fromDegrees(0));
        // }
        // this.setName(autoName);

        // swerve.seedFieldRelative(startPose);

        // this.addCommands(new
        // SetPoint(Constants.ArmConstants.SetPoints.kSubwoofer).withTimeout(Constants.OperatorConstants.setpointTimeout),
        // new FeedAndShoot().withTimeout(0.5),
        // new
        // SetPoint(Constants.ArmConstants.SetPoints.kIntake).withTimeout(Constants.OperatorConstants.setpointTimeout),
        // auto);

    }

    public static void publishTrajectory(String autoName) {
        if (autoName.equals(lastAutoName)) {
            return;
        } else if (autoName.equals("null")) {
            Robot.mField.getObject(Constants.AutoConstants.kFieldObjectName)
                    .setPose(new Pose2d(-5, -5, Rotation2d.fromDegrees(0)));
            return;
        } else {
            lastAutoName = autoName;
        }

        List<Pose2d> poses = new ArrayList<>();
        poses.clear();

        PathPlannerAuto.getPathGroupFromAutoFile(autoName).forEach((path) -> path.getAllPathPoints()
                .forEach((point) -> poses.add(new Pose2d(point.position, point.position.getAngle()))));

        Drivetrain.getInstance().addFieldObj(poses);
    }

    public static void unpublishTrajectory() {
        Drivetrain.getInstance().getField().getObject(Constants.AutoConstants.kFieldObjectName)
                .setPose(new Pose2d(-5, -5, Rotation2d.fromDegrees(0)));
    }
}