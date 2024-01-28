package frc.robot.commands.shooter;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;

public class TargetShoot extends Command {
    private Shooter mShooter;
    private CommandSwerveDrivetrain mDrivetrain;
    private PIDController wristPID = new PIDController(ShooterConstants.wristP, ShooterConstants.wristI, ShooterConstants.wristD);
    private PIDController drivePID = new PIDController(ShooterConstants.aimP, ShooterConstants.aimI, ShooterConstants.aimD);

    private DoubleSupplier leftX, leftY;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(DrivetrainConstants.MaxSpeed * 0.08) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    public TargetShoot(Shooter mShooter, CommandSwerveDrivetrain mDrivetrain, DoubleSupplier leftX, DoubleSupplier leftY){
        this.mShooter = mShooter;
        this.mDrivetrain = mDrivetrain;
        addRequirements(mShooter);
        addRequirements(mDrivetrain);
        this.leftX = leftX;
        this.leftY = leftY;
    }

    private double getDistance(double x, double y) {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)); 
    }

    private void aimWristSpeaker(Pose2d pose) {
        double distance = getDistance(pose.getX()-VisionConstants.speakerCoordinates[0], pose.getY()-VisionConstants.speakerCoordinates[1]);
        double angle = Math.atan(VisionConstants.speakerCoordinates[2]/distance);
        mShooter.setWrist(wristPID.calculate(mShooter.getWristEncoderVal(), angle));
    }

    // TODO: Make sure that 0 radians is pointing away from driver
    private void aimDrivetrainSpeaker(Pose2d pose) {
        double angle = Math.atan((VisionConstants.speakerCoordinates[1]-pose.getY())/(VisionConstants.speakerCoordinates[0]-pose.getX()));
        mDrivetrain.applyRequest(() -> drive.withVelocityX(-leftY.getAsDouble() * DrivetrainConstants.MaxSpeed)
            .withVelocityY(-leftX.getAsDouble() * DrivetrainConstants.MaxSpeed)
            .withRotationalRate(drivePID.calculate(pose.getRotation().getRadians(), angle)));
    }

    public void aimRobotSpeaker() {
        Pose2d pose = mDrivetrain.getState().Pose;
        aimWristSpeaker(pose);
    }
}
