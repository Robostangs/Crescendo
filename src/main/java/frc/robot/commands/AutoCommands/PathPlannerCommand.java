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

    /**
     * A method to publish the trajectory of the autos
     * @param autoName what auto you want to publish 
     */
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

        //if we are calling publish trajectory but the trjectory is already published 
        else if (autoName.equals(lastAutoName)) {
            return;
        }

        //we are going to use the auto name so this is the last auto we published 
        else {
            lastAutoName = autoName;
        }

        List<Pose2d> poses = new ArrayList<>();
        poses.clear();

        try {
            //take the auto and then break it down into paths 
            //and then from the paths break it down into Path points 
            //and then we take the path poses from there
            PathPlannerAuto.getPathGroupFromAutoFile(autoName).forEach((path) -> path.getAllPathPoints()
                    .forEach((point) -> {
                        Pose2d pose = new Pose2d(point.position, point.position.getAngle());
                        if (Robot.isRed()) {
                            pose = GeometryUtil.flipFieldPose(pose);
                        }

                        poses.add(pose);
                    }));
            //flip the poses if we are red
            if (Robot.isRed()) {
                Robot.teleopField.getObject("Starting Pose").setPose(GeometryUtil.flipFieldPose(PathPlannerAuto.getStaringPoseFromAutoFile(autoName)));
            }
            else {
                Robot.teleopField.getObject("Starting Pose").setPose(PathPlannerAuto.getStaringPoseFromAutoFile(autoName));
            }
            
            //none of these are true so these alerts are usless 
            nullAuto.set(false);
            publishfail.set(false);
            noAutoSelected.set(false);
        }
        
        catch (RuntimeException e) {
            //if we call it and we have a null auto name when we are publishing it
            System.out.println("Null Auto: " + autoName);
            nullAuto.setText("Null auto: " + autoName);
            nullAuto.set(true);
        } 
        
        catch (Exception e) {
            //if for some reason it completly dies 
            publishfail.set(true);
            e.printStackTrace();
        }

        Robot.autoField.getObject(Constants.AutoConstants.kFieldObjectName).setPoses(poses);
        Robot.teleopField.getObject(Constants.AutoConstants.kFieldObjectName).setPoses(poses);
    }

    /**
     * a method that uses the {@code publishTrajectory} method and sets it to null 
     * <p>
     *  if we want to unpublish trajectory we set auto name to null and we publish a trajectory to a place where we can't see
     */
    public static void unpublishTrajectory() {
        publishTrajectory(null);
    }
}