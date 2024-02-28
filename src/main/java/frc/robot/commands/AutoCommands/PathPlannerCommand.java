package frc.robot.commands.AutoCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.Swerve.Align;
import frc.robot.subsystems.Drivetrain.Drivetrain;

import java.util.List;
import java.util.ArrayList;

public class PathPlannerCommand extends SequentialCommandGroup {
    private static String lastAutoName;

    public static String[] leftAutos = new String[] { "1 piece", "2 piece", "3 piece" };
    public static String[] rightAutos = new String[] { "1 piece", "2 piece", "3 piece" };
    public static String[] centerAutos = new String[] { "1 piece", "2 piece", "3 piece" };

    public PathPlannerCommand(String autoName, boolean shoot) {
        AutoManager autoManager = new AutoManager();
        Command pathCommand;

        // try {
        //     Robot.mField.getObject("Starting Pose").setPose(PathPlannerAuto.getStaringPoseFromAutoFile(autoName));
        // } catch (Exception e) {
        //     e.printStackTrace();
        // }

        // if there are problems with the alignment of the drivetrain with the note then
        // remove the race with condition, it will disorient the position of the robot
        // cuz the robot isnt expecting to be aligned

        NamedCommands.registerCommand("shoot", new InstantCommand(() -> autoManager.shoot = true)
                .alongWith(new WaitUntilCommand(() -> autoManager.shoot == false)).raceWith(new Align(false)));

        // NamedCommands.registerCommand("shoot", new InstantCommand(() ->
        // autoManager.shoot = true)
        // .alongWith(new WaitUntilCommand(() -> autoManager.shoot == false)));

        try {
            pathCommand = AutoBuilder.buildAuto(autoName);
        } catch (Exception e) {
            e.printStackTrace();
            pathCommand = new PrintCommand("Null Command");
        }

        if (shoot) {
            this.addCommands(
                    new InstantCommand(() -> autoManager.shoot = true)
                            .alongWith(new WaitUntilCommand(() -> autoManager.shoot == false))
                            .andThen(pathCommand).raceWith(autoManager));
        }

        else {
            // TODO: not finishing for some reason
            this.addCommands(
                    pathCommand.raceWith(autoManager));
        }
    }

    public static void publishTrajectory(String autoName) {
        if (autoName == null) {
            Robot.mField.getObject(Constants.AutoConstants.kFieldObjectName)
                    .setPose(new Pose2d(-5, -5, Rotation2d.fromDegrees(0)));
            lastAutoName = "null";
            return;
        } else if (autoName.equals(lastAutoName)) {
            return;
        } else {
            lastAutoName = autoName;
        }

        // if (autoName.equals(lastAutoName)) {
        // return;
        // } else if (autoName.equals("null")) {
        // Robot.mField.getObject(Constants.AutoConstants.kFieldObjectName)
        // .setPose(new Pose2d(-5, -5, Rotation2d.fromDegrees(0)));
        // lastAutoName = "null";
        // return;
        // } else {
        // lastAutoName = autoName;
        // }

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
        } catch (Exception e) {
            e.printStackTrace();
        }

        Drivetrain.getInstance().addFieldObj(poses);
    }

    public static void unpublishTrajectory() {
        Drivetrain.getInstance().getField().getObject(Constants.AutoConstants.kFieldObjectName)
                .setPose(new Pose2d(-5, -5, Rotation2d.fromDegrees(0)));
    }
}