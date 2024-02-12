package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

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
import frc.robot.commands.shooter.AimAndShoot;
import frc.robot.commands.shooter.FeedAndShoot;
import frc.robot.commands.shooter.SetPoint;
import frc.robot.subsystems.Drivetrain.Drivetrain;

public class PathPlannerCommand extends SequentialCommandGroup {
    private Drivetrain swerve;
    private Command auto;
    private Pose2d startPose;
    // private boolean debugMode = true;

    private static String lastAutoName;

    public PathPlannerCommand(String autoName, boolean shoot) {
        NamedCommands.registerCommand("Intake", new PrintCommand("Intake Command, PathPlanner"));
        NamedCommands.registerCommand("Shoot", new PrintCommand("Shoot Command, PathPlanner"));
        // new
        // SetPoint(Constants.ArmConstants.SetPoints.kSpeakerClosestPoint).withTimeout(0.1));
        NamedCommands.registerCommand("AimAndShoot", new AimAndShoot(60).withTimeout(0.3));
        NamedCommands.registerCommand("Stow", new SetPoint(Constants.ArmConstants.SetPoints.kIntake).withTimeout(0.3));

        swerve = Drivetrain.getInstance();
        try {
            startPose = new Pose2d(PathPlannerAuto.getPathGroupFromAutoFile(autoName).get(0)
                    .getPreviewStartingHolonomicPose().getTranslation(), Rotation2d.fromDegrees(0));
            auto = AutoBuilder.buildAuto(autoName);
        } catch (Exception e) {
            this.setName("Null Auto");
            System.out.println("Null auto");
            auto = new PrintCommand("Null Auto");
            startPose = new Pose2d(2, 4, Rotation2d.fromDegrees(0));
        }
        this.setName(autoName);

        swerve.seedFieldRelative(startPose);

        this.addCommands(new SetPoint(Constants.ArmConstants.SetPoints.kSpeakerClosestPoint).withTimeout(0.5),
                new FeedAndShoot().withTimeout(0.5),
                new SetPoint(Constants.ArmConstants.SetPoints.kIntake).withTimeout(0.5), auto);

    }

    public PathPlannerCommand(String startPosition, long closePieces, long farPieces, boolean shoot) {
        List<PathPlannerPath> paths = new ArrayList<>();
        if (shoot) {
            this.addCommands(new SetPoint(Constants.ArmConstants.SetPoints.kSpeakerClosestPoint).withTimeout(0.5),
                    new FeedAndShoot().withTimeout(0.5),
                    new SetPoint(Constants.ArmConstants.SetPoints.kIntake).withTimeout(0.5));
        }

        if (startPosition.equals("left")) {
            if (closePieces == 1) {
                paths.add(PathPlannerPath.fromPathFile("start to close left"));
            }

            if (farPieces == 1) {
                paths.add(PathPlannerPath.fromPathFile("close left to far left"));
            }

            if (farPieces == 2) {
                paths.add(PathPlannerPath.fromPathFile("close left to middle left"));
            }

        } else if (startPosition.equals("right")) {

        } else if (startPosition.equals("center")) {

        } else {
            System.out.println("Invalid start position");
        }

        for (PathPlannerPath path : paths) {
            this.addCommands(AutoBuilder.followPath(path));
        }
    }

    // @Override
    // public void initialize() {
    // System.out.println("Starting Pose: " + startPose.toString());
    // autoGroup = new SequentialCommandGroup(
    // new
    // SetPoint(Constants.ArmConstants.SetPoints.kSpeakerClosestPoint).withTimeout(0.5),
    // new FeedAndShoot().withTimeout(0.5),
    // new SetPoint(Constants.ArmConstants.SetPoints.kIntake).withTimeout(0.5),
    // auto,
    // new InstantCommand(() -> timer.stop()));
    // autoGroup.setName("Autonomous Sequence");
    // timer.restart();
    // autoGroup.schedule();
    // }

    // @Override
    // public void execute() {
    // NetworkTableInstance.getDefault().getTable("PathPlanner").getEntry("Auto
    // Timer").setDouble(timer.get());
    // }

    // @Override
    // public void end(boolean interrupted) {
    // if (interrupted) {
    // auto.cancel();
    // }
    // }

    // @Override
    // public boolean isFinished() {
    // return auto.isFinished();
    // }

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
        Drivetrain.getInstance().getField().getObject(Constants.AutoConstants.kFieldObjectName).close();
        // Swerve.getInstance().getField().getObject(Constants.AutoConstants.kFieldObjectName)
        // .setPose(new Pose2d(-5, -5, Rotation2d.fromDegrees(0)));
    }
}