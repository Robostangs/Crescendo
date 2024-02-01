package frc.robot.commands.shooter;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;

public class TargetShoot extends Command {
    private Shooter mShooter = Shooter.getInstance();
    private CommandSwerveDrivetrain mDrivetrain = CommandSwerveDrivetrain.getInstance();

    private PIDController wristPID = new PIDController(ShooterConstants.WRIST_P, ShooterConstants.WRIST_I, ShooterConstants.WRIST_D);
    private ArmFeedforward wristFeedForward = new ArmFeedforward(ShooterConstants.WRIST_S, ShooterConstants.WRIST_G, ShooterConstants.WRIST_V);
    private PIDController drivePID = new PIDController(ShooterConstants.AIM_P, ShooterConstants.AIM_I, ShooterConstants.AIM_D);

    private Pose2d pose;

    private DoubleSupplier leftX, leftY;
    
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(DrivetrainConstants.MAX_SPEED * 0.08)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    public TargetShoot(DoubleSupplier leftX, DoubleSupplier leftY){
        addRequirements(mShooter);
        addRequirements(mDrivetrain);
        this.leftX = leftX;
        this.leftY = leftY;
    }

    private double getDistance(double x, double y) {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)); 
    }

    private boolean aimWristSpeaker(Pose2d pose) {
        double distance = getDistance(pose.getX()-VisionConstants.SPEAKER_COORDINATES[0], pose.getY()-VisionConstants.SPEAKER_COORDINATES[1]);
        double targetAngle = Math.atan(VisionConstants.SPEAKER_COORDINATES[2]/distance);
        double wristEncoderVal = mShooter.getWristEncoderVal();
        mShooter.setWristVoltage(wristPID.calculate(wristEncoderVal, targetAngle) + wristFeedForward.calculate(wristEncoderVal, mShooter.getWristSpeed())); // TODO: Check with Dan

        return (Math.abs(targetAngle-wristEncoderVal) < ShooterConstants.WRIST_ANGLE_TOLORANCE);
    }

    // TODO: Make sure that 0 radians is pointing away from driver
    private boolean aimDrivetrainSpeaker(Pose2d pose) {
        double targetAngle = Math.atan((VisionConstants.SPEAKER_COORDINATES[1]-pose.getY())/(VisionConstants.SPEAKER_COORDINATES[0]-pose.getX()));
        double robotAngle = pose.getRotation().getRadians();
        mDrivetrain.applyRequest(() -> drive.withVelocityX(-leftY.getAsDouble() * DrivetrainConstants.MAX_SPEED)
            .withVelocityY(-leftX.getAsDouble() * DrivetrainConstants.MAX_SPEED)
            .withRotationalRate(drivePID.calculate(robotAngle, targetAngle)));
        
        return (Math.abs(targetAngle-robotAngle) < ShooterConstants.AIM_ANGLE_TOLORANCE);
    }

    public boolean aimRobotSpeaker() {
        pose = mDrivetrain.getState().Pose;
        return (aimWristSpeaker(pose) && aimDrivetrainSpeaker(pose));
    }

    @Override
    public void execute() {
        mShooter.setShooter(ShooterConstants.SHOOT_SPEED);
        if (aimRobotSpeaker() && mShooter.getAvgRealShootSpeed() > ShooterConstants.SHOOT_SPEED_THRESHOLD)
            mShooter.setFeeder(ShooterConstants.FEED_SPEED);
    }
}
