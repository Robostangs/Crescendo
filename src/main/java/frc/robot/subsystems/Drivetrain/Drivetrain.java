package frc.robot.subsystems.Drivetrain;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import java.util.concurrent.atomic.AtomicInteger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Robot;
import frc.robot.Vision.AprilTagLimelight;
import frc.robot.Vision.LimelightHelpers;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class Drivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005;

    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private Field2d mField;
    private AprilTagLimelight aprilTagReader = new AprilTagLimelight(Constants.Vision.llAprilTag,
            Constants.Vision.llAprilTagRear);
    private BooleanSupplier isRed = () -> {
        if (Robot.atComp) {
            return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        } else {
            return false;
        }
    };

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    @SuppressWarnings("unused")
    @Override
    public void periodic() {
        if (Constants.Vision.UseLimelight && Robot.isReal()) {
            if (LimelightHelpers.getTid(Constants.Vision.llAprilTagRear) != -1) {
                this.addVisionMeasurement(LimelightHelpers.getBotPose2d_wpiBlue(Constants.Vision.llAprilTagRear), Timer.getFPGATimestamp());
            }
        }

        /*
         * This code comes from robotPeriodic() in Robot.java
         * Put this back into Robot.java if the subsystem periodic function seems to
         * fail
         */

        // if (Constants.Vision.UseLimelight && Robot.isReal()) {

        // var lastResult =
        // LimelightHelpers.getLatestResults(Constants.Vision.llAprilTag).targetingResults;
        // Pose2d llPose = lastResult.getBotPose2d_wpiBlue();

        // if (LimelightHelpers.getTid("limelight") != -1) {
        // Drivetrain.getInstance().addVisionMeasurement(llPose,
        // Timer.getFPGATimestamp());
        // }
    }

    private Drivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);

        if (Constants.Vision.UseLimelight) {
            LimelightHelpers.setPipelineIndex(Constants.Vision.llAprilTag, Constants.Vision.llAprilTagPipelineIndex);
            LimelightHelpers.setPipelineIndex(Constants.Vision.llAprilTagRear,
                    Constants.Vision.llAprilTagPipelineIndex);
            LimelightHelpers.setPipelineIndex(Constants.Vision.llPython, Constants.Vision.llPythonPipelineIndex);
        }

        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    private Drivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);

        if (Constants.Vision.UseLimelight) {
            LimelightHelpers.setPipelineIndex(Constants.Vision.llAprilTag, Constants.Vision.llAprilTagPipelineIndex);
            LimelightHelpers.setPipelineIndex(Constants.Vision.llAprilTagRear,
                    Constants.Vision.llAprilTagPipelineIndex);
            LimelightHelpers.setPipelineIndex(Constants.Vision.llPython, Constants.Vision.llPythonPipelineIndex);
        }

        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }

        SmartDashboard.putString("Swerve/.type", "Drivetrain");

        mField = Robot.mField;
    }

    private void configurePathPlanner() {
        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose,
                this::seedFieldRelative,
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)),
                new HolonomicPathFollowerConfig(
                        Constants.AutoConstants.translationPID,
                        Constants.AutoConstants.rotationPID,
                        SwerveConstants.kSpeedAt12VoltsMetersPerSecond,
                        Constants.SwerveConstants.driveBaseRadius,
                        new ReplanningConfig()),
                isRed,
                this);
    }

    /* @Hellothere212 needs to make this a command in the Swerve Commands folder */
    public Command followthePath(Pose2d startPose) {
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                startPose,
                new Pose2d(1.80, 3.33, Rotation2d.fromDegrees(0)),
                new Pose2d(3.46, 0.72, Rotation2d.fromDegrees(0)),
                new Pose2d(6.89, 0.72, Rotation2d.fromDegrees(0)),
                new Pose2d(8.20, 2.30, Rotation2d.fromDegrees(0)));

        PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                new PathConstraints(Constants.SwerveConstants.kSpeedAt12VoltsMetersPerSecond - 1,
                        Constants.SwerveConstants.kSpeedAt12VoltsMetersPerSecond
                                - 1,
                        540d, 720d),
                new GoalEndState(0.0, Rotation2d.fromDegrees(0.0)));

        return AutoBuilder.followPath(path);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public void addFieldObj(PathPlannerTrajectory trajectory) {
        List<Pose2d> poses = new ArrayList<>();
        // int i = 0;
        AtomicInteger i = new AtomicInteger(0);
        trajectory.getStates().forEach((state) -> {
            if (!(state.getTargetHolonomicPose().equals(trajectory.getInitialTargetHolonomicPose()))
                    && i.get() % 10 == 0)
                poses.add(state.getTargetHolonomicPose());
            i.incrementAndGet();
        });
        mField.getObject(Constants.AutoConstants.kFieldObjectName).setPoses(poses);
    }

    public void addFieldObj(List<Pose2d> poses) {
        mField.getObject(Constants.AutoConstants.kFieldObjectName).setPoses(poses);
    }

    public Field2d getField() {
        return mField;
    }

    public Pose2d getPose() {
        return getState().Pose;
    }

    public Constants.SwerveConstants.Target whatAmILookingAt() {
        double rotation = getPose().getRotation().getDegrees();
        if (rotation > -60 && rotation < -120) {
            return Constants.SwerveConstants.Target.kAmp;
        } else {
            return Constants.SwerveConstants.Target.kSpeaker;
        }
    }

    private static Drivetrain mInstance;

    public static Drivetrain getInstance() {
        if (mInstance == null) {
            mInstance = new Drivetrain(SwerveConstants.DrivetrainConstants, SwerveConstants.FrontLeft,
                    SwerveConstants.FrontRight, SwerveConstants.BackLeft, SwerveConstants.BackRight);
        }
        return mInstance;
    }

}
