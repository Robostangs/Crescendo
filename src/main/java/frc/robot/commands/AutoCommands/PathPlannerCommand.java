package frc.robot.commands.AutoCommands;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.Robot;

import java.util.List;
import java.util.ArrayList;

public class PathPlannerCommand {
    private static String lastAutoName;

    public static void publishTrajectory(String autoName) {
        if (autoName == null) {
            Robot.autoField.getObject(Constants.AutoConstants.kFieldObjectName)
                    .setPose(new Pose2d(-5, -5, Rotation2d.fromDegrees(0)));
            Robot.teleopField.getObject(Constants.AutoConstants.kFieldObjectName)
                    .setPose(new Pose2d(-5, -5, Rotation2d.fromDegrees(0)));
            lastAutoName = "null";
            return;
        } else if (autoName.equals(lastAutoName)) {
            return;
        } else {
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
        } catch (Exception e) {
            e.printStackTrace();
        }

        Robot.autoField.getObject(Constants.AutoConstants.kFieldObjectName).setPoses(poses);
        Robot.teleopField.getObject(Constants.AutoConstants.kFieldObjectName).setPoses(poses);
    }

    public static void unpublishTrajectory() {
        publishTrajectory(null);
        // Drivetrain.getInstance().getField().getObject(Constants.AutoConstants.kFieldObjectName)
        //         .setPose(new Pose2d(-5, -5, Rotation2d.fromDegrees(0)));
    }
}