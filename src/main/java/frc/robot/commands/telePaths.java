package frc.robot.commands;

import java.util.List;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Telemetry;
import frc.robot.generated.TunerConstants;

public class telePaths extends Command{
    private CommandSwerveDrivetrain m_drive;
    private TunerConstants m_tunerConstants;
    private Telemetry m_telemetry;


    private Pose2d currentPose = new Pose2d();
    private Pose2d targetPose = new Pose2d();
    private SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                    .withDeadband(m_tunerConstants.kSpeedAt12VoltsMps * 0.08)
                    .withRotationalDeadband(0)
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public telePaths() {}
                
                    
    public telePaths(Pose2d currentPose) {
        this.currentPose = currentPose;

    }


    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
    currentPose,
    //TODO add proper waypoints
    new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
    new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90))
);

// Create the path using the bezier points created above
PathPlannerPath path = new PathPlannerPath(
    bezierPoints,
    //TODO add proper constraints
    new PathConstraints(0, 0, 0, 0), 
    //TODO add proper goal
    new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) 
);

//TODO get ChassisSpped and Rotation2d 
PathPlannerTrajectory trajectory = new PathPlannerTrajectory(path,new ChassisSpeeds(0, 0, 0),new Rotation2d(0));
PathPlannerTrajectory.State state = trajectory.getState(0);


  
//my idea is to have many methods that have different paths
public void Path1() {

bezierPoints = PathPlannerPath.bezierFromPoses(
    currentPose,
    //TODO add proper waypoints
    new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
    new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90))
);

// Create the path using the bezier points created above
 path = new PathPlannerPath(
    bezierPoints,
    //TODO add proper constraints
    new PathConstraints(0, 0, 0, 0), 
    //TODO add proper goal
    new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) 
);

//TODO get ChassisSpped and Rotation2d 
 trajectory = new PathPlannerTrajectory(path, new ChassisSpeeds(0, 0, 0),new Rotation2d(0));

}


public void execute() {
   
    targetPose = state.getTargetHolonomicPose();
    m_drive.applyRequest(() ->drive.withVelocityX(0).withVelocityY(0).withRotationalRate(targetPose.getRotation().getRadians()));

    

 

}
}
