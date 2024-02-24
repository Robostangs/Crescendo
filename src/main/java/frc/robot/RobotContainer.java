package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Spit;
import frc.robot.commands.Swerve.Align;
import frc.robot.commands.Swerve.PathToPoint;
import frc.robot.commands.Swerve.xDrive;
import frc.robot.commands.feeder.BeltDrive;
import frc.robot.commands.feeder.BeltFeed;
import frc.robot.commands.feeder.DeployAndIntake;
import frc.robot.commands.feeder.QuickFeed;
import frc.robot.commands.feeder.ShooterCharge;
import frc.robot.commands.shooter.AimAndShoot;
import frc.robot.commands.shooter.FeedAndShoot;
import frc.robot.commands.shooter.FineAdjust;
import frc.robot.commands.shooter.SetPoint;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Drivetrain.Drivetrain;

@SuppressWarnings("unused")
public class RobotContainer {
	private final CommandXboxController xDrive = new CommandXboxController(0);
	private final CommandXboxController xManip = new CommandXboxController(1);
	private final GenericHID simController = new GenericHID(3);

	private final Drivetrain drivetrain = Drivetrain.getInstance();
	private final Arm mArm = Arm.getInstance();
	private final Intake mIntake = Intake.getInstance();

	private final Telemetry logger;
	public Field2d field;

	private void configureDriverBinds() {
		mIntake.setDefaultCommand(new BeltFeed());

		drivetrain.setDefaultCommand(
				new xDrive(xDrive::getLeftX, xDrive::getLeftY, xDrive::getRightX,
						xDrive::getRightTriggerAxis).ignoringDisable(true));

		xDrive.a().toggleOnTrue(new Align(xDrive::getLeftX, xDrive::getLeftY,
				xDrive::getRightTriggerAxis, false));
		xDrive.b().toggleOnTrue(new Align(xDrive::getLeftX, xDrive::getLeftY,
				xDrive::getRightTriggerAxis, true));
		xDrive.x().toggleOnTrue(new AimAndShoot());

		// xDrive.x().toggleOnTrue(new
		// PathToPoint(Constants.AutoConstants.WayPoints.Blue.kAmp));

		// Square up to the speaker and press this to reset odometry to the speaker
		// pose, this is consistent af
		xDrive.povRight().onTrue(drivetrain
				.runOnce(() -> drivetrain.seedFieldRelative(new Pose2d(1.25, 5.55, Rotation2d.fromDegrees(0)))));
		xDrive.povDown().onTrue(drivetrain.runOnce(drivetrain::seedFieldRelative));

		// This is basically saying deploy and intake without deploying
		xDrive.rightBumper().onTrue(mIntake.runOnce(() -> mIntake.setHolding(!mIntake.getHolding())));
		xDrive.leftBumper().toggleOnTrue(new DeployAndIntake());

	}

	private void configureManipBinds() {
		new Trigger(() -> Math.abs(xManip.getRightY()) > Constants.OperatorConstants.kManipDeadzone)
				.whileTrue(new FineAdjust(() -> -xManip.getRightY()));

		new Trigger(() -> xManip.getLeftTriggerAxis() > Constants.OperatorConstants.kManipDeadzone)
				.or(() -> xManip.getRightTriggerAxis() > Constants.OperatorConstants.kManipDeadzone)
				.whileTrue(new FineAdjust(() -> xManip.getRightTriggerAxis() - xManip.getLeftTriggerAxis()));

		new Trigger(() -> Math.abs(xManip.getLeftY()) > Constants.OperatorConstants.kManipDeadzone)
				.whileTrue(new BeltDrive(() -> -xManip.getLeftY()));

		new Trigger(() -> xManip.getRightTriggerAxis() > Constants.OperatorConstants.kManipDeadzone)
				.whileTrue(new ShooterCharge(xManip::getRightTriggerAxis));

		xManip.y().toggleOnTrue(new SetPoint());
		xManip.x().whileTrue(new QuickFeed());

		xManip.a().toggleOnTrue(new AimAndShoot(() -> xManip.getHID().getLeftBumper()));
		xManip.b().toggleOnTrue(
				new AimAndShoot(Constants.ArmConstants.SetPoints.kAmp, () -> xManip.getHID().getLeftBumper()));

		xManip.povRight().whileTrue(new Spit());
		xManip.povDown().whileTrue(new DeployAndIntake());

		xManip.rightBumper().whileTrue(new FeedAndShoot(() -> xManip.getHID().getLeftBumper()));
		xManip.leftBumper().whileTrue(new FeedAndShoot());

		// absolute worst case scenario
		xManip.start().and(() -> xManip.back().getAsBoolean())
				.onTrue(mArm.runOnce(mArm::toggleArmMotorLimits));
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
		drivetrain.removeDefaultCommand();
		mArm.removeDefaultCommand();

		drivetrain.setDefaultCommand(new xDrive(() -> simController.getRawAxis(0), () -> simController.getRawAxis(1),
				() -> simController.getRawAxis(2), () -> 0d));

		new Trigger(() -> simController.getRawButtonPressed(1))
				.whileTrue(new SetPoint(Constants.ArmConstants.SetPoints.kSubwoofer));

		new Trigger(() -> simController.getRawButtonPressed(2))
				.whileTrue(new SetPoint(Constants.ArmConstants.SetPoints.kAmp));

		new Trigger(() -> simController.getRawButtonPressed(3))
				.whileTrue(new AimAndShoot());

		new Trigger(() -> simController.getRawButtonPressed(4))
				.toggleOnTrue(new PathToPoint(Constants.AutoConstants.WayPoints.Blue.kAmp));

		// new Trigger(() -> simController.getRawButtonPressed(3))
		// .whileTrue(new AimAndShoot());

		// new Trigger(() -> simController.getRawButtonPressed(4))
		// .whileTrue(new SetPoint(Constants.ArmConstants.SetPoints.kHorizontal));

		// new Trigger(() -> simController.getRawButtonPressed(4)).onTrue(new
		// InstantCommand(() -> mArm.calculateArmSetpoint(), mArm));

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
