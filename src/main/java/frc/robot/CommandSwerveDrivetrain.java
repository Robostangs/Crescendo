package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private Field2d mField = new Field2d();



    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);

        if (Utils.isSimulation()) {
            startSimThread();
        }

    }

    public Command followthePath(Pose2d startPose) {
        this.applyRequest(() -> drive.withRotationalDeadband(this.getState().Pose.getRotation().getDegrees() - this.getState().Pose.getRotation().getDegrees()));


        // List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
        //         new Pose2d(startPose.getTranslation(), Rotation2d.fromDegrees(-90)),
        //         new Pose2d(4.54, 4.05, Rotation2d.fromDegrees(-82.76)),
        //         new Pose2d(5, 0.6, Rotation2d.fromDegrees(1.86)),
        //         new Pose2d(8.22, 2.34, Rotation2d.fromDegrees(52.16)));

        // PathPlannerPath paths = new PathPlannerPath(
        //         bezierPoints,
        //         new PathConstraints(5, 6, 540d, 720d),
        //         new GoalEndState(0.0, Rotation2d.fromDegrees(-90.0)));

        PathConstraints constraints = new PathConstraints(5, 6, 540d, 720d);

  
        Pose2d targetPose = new Pose2d(8.27, 2.42, Rotation2d.fromDegrees(0));

        Command pathfindingCommand = AutoBuilder.pathfindToPose(
        targetPose,
        constraints,
        0.0, // Goal end velocity in meters/sec
        0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
);
        return pathfindingCommand;
      

    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);

        HolonomicPathFollowerConfig FollowConfig = new HolonomicPathFollowerConfig(
                // new PIDConstants(0.0001, 1.25, 0.4), // Translation PID constants
                new PIDConstants(0, 0, 0), // Translation PID constants
                new PIDConstants(1.75, 0.0, 0.0), // Rotation PID constants
                // new PIDConstants(12.5, 0.5, 0.3), // Translation PID constants
                // new PIDConstants(1.57,0.07,0.9,1), // Rotation PID constants
                TunerConstants.kSpeedAt12VoltsMps, // Max module speed, in m/s
                Units.inchesToMeters(Math.sqrt(Math.pow(14.75, 2) + Math.pow(14.75, 2))),
                new ReplanningConfig()); // Default path replanning config. See the API for the options here

        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose, // Robot pose supplier
                // () -> new Pose2d(4.54, 4.05, Rotation2d.fromDegrees(-90)),
                (pose) -> this.seedFieldRelative(pose), // Method to reset odometry (will be called if your auto has a starting pose)
                () -> getChassisSpeeds(), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds) -> setControl(// Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                        drive.withVelocityX(speeds.vxMetersPerSecond).withVelocityY(speeds.vyMetersPerSecond).withRotationalRate(speeds.omegaRadiansPerSecond)),
                FollowConfig,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
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

    @Override
    public void periodic() {

        //
        mField.setRobotPose(this.getState().Pose);

        SmartDashboard.putData("Field", mField);


        SmartDashboard.putNumber("robot angle", this.getState().Pose.getRotation().getDegrees());

        SmartDashboard.putNumber("robot x", -(this.getState().Pose.getRotation().getDegrees()+1));

   

    }
    

}
