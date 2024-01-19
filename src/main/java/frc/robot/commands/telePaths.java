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
    currentPose
);

// Create the path using the bezier points created above
PathPlannerPath path = new PathPlannerPath(
    bezierPoints,
    new PathConstraints(0, 0, 0, 0), 
    new GoalEndState(0.0, Rotation2d.fromDegrees(0)) 
);

PathPlannerTrajectory trajectory = new PathPlannerTrajectory(path,new ChassisSpeeds(0, 0, 0),new Rotation2d(0));
PathPlannerTrajectory.State state = trajectory.getState(0);


  
//my idea is to have many methods that have different paths
public void Path1() {

bezierPoints = PathPlannerPath.bezierFromPoses(
    new Pose2d(1.80, 3.33, Rotation2d.fromDegrees(0)),
    new Pose2d(3.46, 0.72, Rotation2d.fromDegrees(0)),
    new Pose2d(6.89, 0.72, Rotation2d.fromDegrees(0)),
    new Pose2d(8.20, 2.30, Rotation2d.fromDegrees(0))
);

// Create the path using the bezier points created above
 path = new PathPlannerPath(
    bezierPoints,
    new PathConstraints(3d, 3d, 540d, 720d), 
    new GoalEndState(0.0, Rotation2d.fromDegrees(0.0)) 
);

//TODO get ChassisSpped and Rotation2d 
 trajectory = new PathPlannerTrajectory(path, new ChassisSpeeds(3.00, 0.00, 0),new Rotation2d(0));

}


public void execute() {
   
    targetPose = state.getTargetHolonomicPose();
    m_drive.applyRequest(() ->drive.withVelocityX(3.00).withVelocityY(3.00).withRotationalRate(targetPose.getRotation().getRadians()));

    

 

}
}
