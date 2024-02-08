package frc.robot.Commands;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Commands.Shooter.AimAndShoot;
import frc.robot.Commands.Shooter.FeedAndShoot;
import frc.robot.Commands.Shooter.SetPoint;
import frc.robot.Subsystems.Drivetrain;

public class PathPlannerCommand extends Command {
    private Drivetrain swerve;
    private Command auto;
    private Pose2d startPose;
    private SequentialCommandGroup autoGroup;
    // private boolean debugMode = true;
    private Timer timer;

    private static String lastAutoName;

    public PathPlannerCommand(String autoName, boolean shoot) {
        NamedCommands.registerCommand("Intake", new PrintCommand("Intake Command, PathPlanner"));
        NamedCommands.registerCommand("Shoot", new PrintCommand("Shoot Command, PathPlanner"));
        NamedCommands.registerCommand("AimAndShoot", new AimAndShoot().withTimeout(0.3));
        NamedCommands.registerCommand("Stow", new SetPoint(Constants.ArmConstants.SetPoints.kIntake).withTimeout(0.3));

        swerve = Drivetrain.getInstance();
        timer = new Timer();

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
    }

    @Override
    public void initialize() {
        swerve.seedFieldRelative(startPose);
        System.out.println("Starting Pose: " + startPose.toString());
        autoGroup = new SequentialCommandGroup(
                new SetPoint(Constants.ArmConstants.SetPoints.kSpeakerClosestPoint).withTimeout(0.5),
                new FeedAndShoot().withTimeout(0.5),
                new SetPoint(Constants.ArmConstants.SetPoints.kIntake).withTimeout(0.5), auto,
                new InstantCommand(() -> timer.stop()));
        autoGroup.setName("Autonomous Sequence");
        timer.restart();
        autoGroup.schedule();
    }

    @Override
    public void execute() {
        NetworkTableInstance.getDefault().getTable("PathPlanner").getEntry("Auto Timer").setDouble(timer.get());
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            auto.cancel();
        }
    }

    @Override
    public boolean isFinished() {
        return auto.isFinished();
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
        Drivetrain.getInstance().getField().getObject(Constants.AutoConstants.kFieldObjectName).close();
        // Swerve.getInstance().getField().getObject(Constants.AutoConstants.kFieldObjectName)
        //         .setPose(new Pose2d(-5, -5, Rotation2d.fromDegrees(0)));
    }
}