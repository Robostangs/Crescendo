package frc.robot.commands.AutoCommands;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Alert;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Alert.AlertType;

import java.util.List;
import java.util.ArrayList;

public class PathPlannerCommand {
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
                Robot.teleopField.getObject("Starting Pose").setPose(GeometryUtil.flipFieldPose(PathPlannerAuto.getStaringPoseFromAutoFile(autoName)));
            }

            else {
                Robot.teleopField.getObject("Starting Pose").setPose(PathPlannerAuto.getStaringPoseFromAutoFile(autoName));
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