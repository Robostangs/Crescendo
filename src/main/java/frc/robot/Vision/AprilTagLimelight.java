package frc.robot.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Subsystems.Drivetrain.Drivetrain;

public class AprilTagLimelight {
    private String limelightOne, limelightTwo;

    public AprilTagLimelight(String limelightOne, String limelightTwo) {
        this.limelightOne = limelightOne;
        this.limelightTwo = limelightTwo;
    }

    public Pose2d getPoseAvg() {
        int tidOne = (int) LimelightHelpers.getTid(limelightOne);
        int tidTwo = (int) LimelightHelpers.getTid(limelightTwo);

        Pose2d llPoseOne = LimelightHelpers.getLatestResults(limelightOne).targetingResults.getBotPose2d_wpiBlue();
        Pose2d llPoseTwo = LimelightHelpers.getLatestResults(limelightTwo).targetingResults.getBotPose2d_wpiBlue();

        if (tidOne != -1 && tidTwo != -1) {
            double xAvg = (llPoseOne.getX() + llPoseTwo.getX())/2;
            double yAvg = (llPoseOne.getY() + llPoseTwo.getY())/2;
            double angleAvg = (llPoseOne.getRotation().getRadians() + llPoseTwo.getRotation().getRadians())/2;
            return new Pose2d(xAvg, yAvg, new Rotation2d(angleAvg));
        } else if (tidOne != -1) {
            return llPoseOne;
        } else if (tidTwo != -1) {
            return llPoseTwo;
        } else {
            System.out.println("No AprilTag pose");
            return Drivetrain.getInstance().getPose();
        }
    }
}
