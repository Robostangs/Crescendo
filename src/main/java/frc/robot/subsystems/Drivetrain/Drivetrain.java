package frc.robot.subsystems.Drivetrain;

import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2FeaturesConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import java.util.concurrent.atomic.AtomicInteger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Robot;
import frc.robot.Vision.LimelightHelpers;
import frc.robot.Vision.LimelightHelpers.LimelightResults;
import frc.robot.Vision.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.Music;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@SuppressWarnings("unused")
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

    public final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    @Override
    public void periodic() {
        super.setOperatorPerspectiveForward(Rotation2d.fromDegrees((Robot.isRed() ? 180 : 0)));

        if (Constants.Vision.UseLimelight && Robot.isReal()) {

            PoseEstimate frontPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.Vision.llAprilTag);

            if (frontPoseEstimate.tagCount > 1 && frontPoseEstimate.avgTagArea > 0.4) {
                this.addVisionMeasurement(frontPoseEstimate.pose,
                        Timer.getFPGATimestamp() - frontPoseEstimate.latency / 1000);
            }

            PoseEstimate rearPoseEstimate = LimelightHelpers
                    .getBotPoseEstimate_wpiBlue(Constants.Vision.llAprilTagRear);

            if (rearPoseEstimate.tagCount > 1 && rearPoseEstimate.avgTagArea > 0.4) {
                this.addVisionMeasurement(rearPoseEstimate.pose,
                        Timer.getFPGATimestamp() - rearPoseEstimate.latency / 1000);
            }

            // LimelightResults rearResults =
            // LimelightHelpers.getLatestResults(Constants.Vision.llAprilTagRear);
            // int howManyTagsFront =
            // NetworkTableInstance.getDefault().getTable(Constants.Vision.llAprilTag)

            // if (LimelightHelpers.getTA(Constants.Vision.llAprilTagRear) > 0.26
            // && rearResults.targetingResults.targets_Fiducials.length > 1) {
            // this.addVisionMeasurement(rearResults.targetingResults.getBotPose2d_wpiBlue(),
            // Timer.getFPGATimestamp() - rearResults.targetingResults.latency_pipeline /
            // 1000);
            // }

            // // LimelightResults frontResults =
            // LimelightHelpers.getLatestResults(Constants.Vision.llAprilTag);

            // if (LimelightHelpers.getTA(Constants.Vision.llAprilTag) > 0.26
            // && frontResults.targetingResults.targets_Fiducials.length > 1) {
            // this.addVisionMeasurement(frontResults.targetingResults.getBotPose2d_wpiBlue(),
            // Timer.getFPGATimestamp() - frontResults.targetingResults.latency_pipeline /
            // 1000);
            // }

            mField.getObject("Rear LL pose").setPose(rearPoseEstimate.pose);
            mField.getObject("Front LL pose").setPose(frontPoseEstimate.pose);

        }

        SmartDashboard.putBoolean("Swerve/Is In Range", isInRangeOfTarget());
        SmartDashboard.putNumber("Swerve/Rotation Error", (angleToSpeaker() -
                getPose().getRotation().getDegrees()));

    }

    private final SwerveRequest.SysIdSwerveTranslation TranslationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveRotation RotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
    private final SwerveRequest.SysIdSwerveSteerGains SteerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();

    /* Use one of these sysidroutines for your particular test */
    private SysIdRoutine SysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(4),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> setControl(TranslationCharacterization.withVolts(volts)),
                    null,
                    this));

    private final SysIdRoutine SysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(4),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> setControl(RotationCharacterization.withVolts(volts)),
                    null,
                    this));
    private final SysIdRoutine SysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(7),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> setControl(SteerCharacterization.withVolts(volts)),
                    null,
                    this));

    /* Change this to the sysid routine you want to test */
    private final SysIdRoutine RoutineToApply = SysIdRoutineTranslation;

    private Drivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);

        if (Constants.Vision.UseLimelight) {
            LimelightHelpers.setPipelineIndex(Constants.Vision.llAprilTag,
                    Constants.Vision.llAprilTagPipelineIndex);
            LimelightHelpers.setPipelineIndex(Constants.Vision.llAprilTagRear,
                    Constants.Vision.llAprilTagPipelineIndex);
            LimelightHelpers.setPipelineIndex(Constants.Vision.llPython, Constants.Vision.llPythonPipelineIndex);

            super.setVisionMeasurementStdDevs(Constants.Vision.kPrecisionInMyVision);
        }

        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }

        mField = Robot.teleopField;
        if (Robot.isRed()) {
            mField.getObject("Speaker").setPose(Constants.Vision.SpeakerPoseRed);
        }

        else {
            mField.getObject("Speaker").setPose(Constants.Vision.SpeakerPoseBlue);
        }

        for (SwerveModule module : Modules) {
            Music.getInstance().addFalcon(module.getDriveMotor(), module.getSteerMotor());
        }
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
                        SwerveConstants.maxModuleSpeed,
                        Constants.SwerveConstants.driveBaseRadius,
                        new ReplanningConfig(true, false, 1, 0.25)),
                Robot::isRed,
                this);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /*
     * Both the sysid commands are specific to one particular sysid routine, change
     * which one you're trying to characterize
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return RoutineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return RoutineToApply.dynamic(direction);
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

    // ankur is mine hehehehhehehehehehehhehe
    public Pose2d getPose() {
        Pose2d pose = getState().Pose;

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

    public double getDistanceToSpeaker() {
        Pose2d speakerPose;

        if (Robot.isRed()) {
            speakerPose = Constants.Vision.SpeakerPoseRed;
        } else {
            speakerPose = Constants.Vision.SpeakerPoseBlue;
        }

        /* Swerve Pose calculated in meters */
        Pose2d currentPose = Drivetrain.getInstance().getPose();
        double SpeakerY = speakerPose.getY();

        double Ydeadband = 1;

        double distToSpeakerMeters = Math.sqrt(
                Math.pow(speakerPose.getX() - currentPose.getX(), 2)
                        + Math.pow(SpeakerY - currentPose.getY(), 2));

        return Math.abs(distToSpeakerMeters);
    }

    public double angleToSpeaker() {
        if (Robot.isRed()) {
            return Rotation2d
                    .fromRadians(Math.atan2(
                            getPose().getY() - Constants.Vision.SpeakerPoseRed.getY(),
                            getPose().getX() - Constants.Vision.SpeakerPoseRed.getX()))
                    .getDegrees();
        }

        else {
            return Rotation2d
                    .fromRadians(Math.atan2(
                            getPose().getY() - Constants.Vision.SpeakerPoseBlue.getY(),
                            getPose().getX() - Constants.Vision.SpeakerPoseBlue.getX()))
                    .getDegrees();
        }
    }

    /**
     * Whether the robot is within a certain range of the speaker
     * 
     * @param range the range in degrees
     * @return true if the robot is within the range, false otherwise
     */
    public boolean isInRangeOfTarget(double range) {
        return Math.abs(angleToSpeaker() - getPose().getRotation().getDegrees()) < range;
    }

    /**
     * Whether the robot is within 15 degrees of the speaker
     * 
     * @return true if the robot is within 15 degrees, false otherwise
     */
    public boolean isInRangeOfTarget() {
        return isInRangeOfTarget(8);
    }

    public boolean readyToShoot() {
        return this.isInRangeOfTarget() &&
                Math.abs(this.getState().speeds.omegaRadiansPerSecond) < 0.05;
    }

    private static Drivetrain mInstance;

    public static Drivetrain getInstance() {
        if (mInstance == null) {
            mInstance = new Drivetrain(SwerveConstants.TunerConstants.DrivetrainConstants,
                    SwerveConstants.TunerConstants.FrontLeft,
                    SwerveConstants.TunerConstants.FrontRight, SwerveConstants.TunerConstants.BackLeft,
                    SwerveConstants.TunerConstants.BackRight);
        }
        return mInstance;
    }
}
