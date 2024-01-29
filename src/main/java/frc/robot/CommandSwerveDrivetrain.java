package frc.robot;

import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(TunerConstants.kSpeedAt12VoltsMps * 0.08)
            .withRotationalDeadband(0)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        // AutoBuilder.configureHolonomic(null, null, null, null, null, null, null);
        AutoBuilder.configureHolonomic(
            () -> this.getState().Pose, // Robot pose supplier
            (pose) -> this.seedFieldRelative(pose), // Method to reset odometry (will be called if your auto has a starting pose)
            () -> getChassisSpeeds(), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds) -> setControl(drive.withVelocityX(speeds.vxMetersPerSecond).withVelocityY(speeds.vyMetersPerSecond)),// Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                   TunerConstants.FollowConfig,
                    () -> {
                        // Boolean supplier that controls when the path will be mirrored for the red alliance
                        // This will flip the path being followed to the red side of the field.
                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
    
                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    this // Reference to this subsystem to set requirements
            );
    
 

    }

    public Command followPath(Supplier<Pose2d> startPose){

        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
            startPose.get(),
            new Pose2d(1.80, 3.33, Rotation2d.fromDegrees(0)),
            new Pose2d(3.46, 0.72, Rotation2d.fromDegrees(0)),
            new Pose2d(6.89, 0.72, Rotation2d.fromDegrees(0)),
            new Pose2d(8.20, 2.30, Rotation2d.fromDegrees(0)));

    PathPlannerPath path = new PathPlannerPath(
            bezierPoints,
            new PathConstraints(3d, 3d, 540d, 720d),
            new GoalEndState(0.0, Rotation2d.fromDegrees(0.0)));

            return AutoBuilder.followPath(path);
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
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

    public ChassisSpeeds getChassisSpeeds() {
        return TunerConstants.DriveTrain.m_kinematics
                .toChassisSpeeds(TunerConstants.DriveTrain.m_cachedState.ModuleStates);
    }
       

}
