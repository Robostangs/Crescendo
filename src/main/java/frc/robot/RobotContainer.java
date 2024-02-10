package frc.robot;

import java.util.Set;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Spit;
import frc.robot.commands.feeder.DeployAndIntake;
import frc.robot.commands.shooter.AimAndShoot;
import frc.robot.commands.shooter.FeedAndShootVelocity;
import frc.robot.commands.shooter.FineAdjust;
import frc.robot.commands.shooter.SetPoint;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.subsystems.Drivetrain.SwerveRequest;
import frc.robot.subsystems.Drivetrain.SwerveModule.DriveRequestType;

public class RobotContainer {
	private final CommandXboxController xDrive = new CommandXboxController(0);
	private final CommandXboxController xManip = new CommandXboxController(1);
	private final GenericHID simController = new GenericHID(3);

	private final Drivetrain drivetrain = Drivetrain.getInstance();
	private final Arm mArm = Arm.getInstance();

	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
			.withDeadband(Constants.OperatorConstants.deadband)
			.withRotationalDeadband(Constants.OperatorConstants.rotationalDeadband)
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

	private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
			.withDeadband(Constants.OperatorConstants.deadband)
			.withRotationalDeadband(Constants.OperatorConstants.rotationalDeadband)
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	private final Telemetry logger;
	public Field2d field;

	private void configureDriverBinds() {
		drivetrain.setDefaultCommand(
				drivetrain.applyRequest(() -> drive
						.withVelocityX(-xDrive.getLeftY()
								* Constants.SwerveConstants.kMaxSpeedMetersPerSecond)
						.withVelocityY(-xDrive.getLeftX()
								* Constants.SwerveConstants.kMaxSpeedMetersPerSecond)
						.withRotationalRate(
								-xDrive.getRightX()
										* Constants.SwerveConstants.kMaxAngularSpeedMetersPerSecond)
						.withSlowDown(true, 1 - xDrive.getRightTriggerAxis()))
						.ignoringDisable(true));

		xDrive.x().whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(1).withVelocityY(1)));
		xDrive.a().whileTrue(drivetrain.applyRequest(() -> brake));
		xDrive.b().whileTrue(drivetrain
				.applyRequest(() -> point.withModuleDirection(
						new Rotation2d(-xDrive.getLeftY(), xDrive.getLeftX()))));

		

		xDrive.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
		xDrive.rightBumper().whileTrue(
				new DeferredCommand(() -> drivetrain.followthePath(drivetrain.getState().Pose), Set.of(drivetrain)));
	}

	private void configureManipBinds() {
		mArm.setDefaultCommand(new FineAdjust(() -> -xManip.getRightY()));
		xManip.x().onTrue(new SetPoint(Constants.ArmConstants.SetPoints.kSpeakerClosestPoint));
		xManip.y().whileTrue(new AimAndShoot(0));
		xManip.a().onTrue(new SetPoint(Constants.ArmConstants.SetPoints.kAmp));
		xManip.b().onTrue(new SetPoint(0));

		xManip.pov(90).whileTrue(new Spit());
		xManip.pov(180).whileTrue(new DeployAndIntake());

		xManip.leftBumper().whileTrue(new FeedAndShootVelocity(() -> xManip.getHID().getRightBumper()));
	}

	public RobotContainer() {
		logger = new Telemetry();
		field = Robot.mField;
		drivetrain.registerTelemetry((telemetry) -> logger.telemeterize(telemetry));
		configureDriverBinds();
		configureManipBinds();
		if (Robot.isSimulation()) {
			configureSimBinds();
		}
	}

	private void configureSimBinds() {
		drivetrain.setDefaultCommand(drivetrain
				.applyRequest(() -> drive
						.withVelocityX(
								-simController.getRawAxis(1)
										* Constants.SwerveConstants.kMaxSpeedMetersPerSecond)
						.withVelocityY(
								-simController.getRawAxis(0)
										* Constants.SwerveConstants.kMaxSpeedMetersPerSecond)
						.withRotationalRate(-simController.getRawAxis(2)
								* Constants.SwerveConstants.kMaxAngularSpeedMetersPerSecond)
						.withSlowDown(simController.getRawButton(5),
								Constants.SwerveConstants.slowDownMultiplier))
				.withName("xDrive"));

		new Trigger(() -> simController.getRawButtonPressed(1))
				.whileTrue(new SetPoint(Constants.ArmConstants.SetPoints.kSpeakerClosestPoint));

		new Trigger(() -> simController.getRawButtonPressed(2))
				.whileTrue(new SetPoint(Constants.ArmConstants.SetPoints.kAmp));

		new Trigger(() -> simController.getRawButtonPressed(3))
				.whileTrue(new SetPoint(Constants.ArmConstants.SetPoints.kSpeaker));

		new Trigger(() -> simController.getRawButtonPressed(4))
				.whileTrue(new SetPoint(Constants.ArmConstants.SetPoints.kHorizontal));

		// new Trigger(() -> simController.getRawButtonPressed(2))
		// .onTrue(new InstantCommand(() -> drivetrain.seedFieldRelative()));

		// new Trigger(() -> simController.getRawButtonPressed(3))
		// .onTrue(new
		// SetPoint(Constants.ArmConstants.SetPoints.kSpeakerClosestPoint).withTimeout(1));

		// new Trigger(() -> simController.getRawButtonPressed(3)).onTrue(new
		// SetPoint(0).withTimeout(1));

		// new Trigger(() -> simController.getRawButtonPressed(3))
		// .whileTrue(drivetrain.applyRequest(() ->
		// forwardStraight.withVelocityX(1).withVelocityY(0)));

		// mArm.setDefaultCommand(new FineAdjust(() -> -simController.getRawAxis(2)));

		// new Trigger(() -> simController.getRawButtonPressed(4)).onTrue(new
		// InstantCommand(Intake.toggleDeploy));
	}
}