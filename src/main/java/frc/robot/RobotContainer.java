package frc.robot;

import com.ctre.phoenix6.signals.Licensing_IsSeasonPassedValue;
import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Lights.LEDState;
import frc.robot.commands.ShootCommandFactory;
import frc.robot.commands.Spit;
import frc.robot.commands.ArmCommands.FineAdjust;
import frc.robot.commands.ArmCommands.ReturnHome;
import frc.robot.commands.ArmCommands.SetPoint;
import frc.robot.commands.ClimberCommands.ClimberAdjust;
import frc.robot.commands.ClimberCommands.Extend;
import frc.robot.commands.ClimberCommands.HomeClimber;
import frc.robot.commands.ClimberCommands.Retract;
import frc.robot.commands.FeederCommands.BeltDrive;
import frc.robot.commands.FeederCommands.PassToShooter;
import frc.robot.commands.FeederCommands.QuickFeed;
import frc.robot.commands.IntakeCommands.DeployAndIntake;
import frc.robot.commands.IntakeCommands.MultiIntake;
import frc.robot.commands.ShooterCommands.CancelShooter;
import frc.robot.commands.ShooterCommands.Feed;
import frc.robot.commands.ShooterCommands.Prepare;
import frc.robot.commands.ShooterCommands.Shoot;
import frc.robot.commands.Swerve.AlignToSpeaker;
import frc.robot.commands.Swerve.AlignToAmp;
import frc.robot.commands.Swerve.PathToPoint;
import frc.robot.commands.Swerve.xDrive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.Music;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivetrain.Drivetrain;

@SuppressWarnings("unused")
public class RobotContainer {
	public static final CommandXboxController xDrive = new CommandXboxController(0);
	public static final CommandXboxController xManip = new CommandXboxController(1);
	public static final CommandXboxController xPit = new CommandXboxController(2);
	private static final GenericHID simController = new GenericHID(3);

	private final Drivetrain drivetrain = Drivetrain.getInstance();
	private final Arm arm = Arm.getInstance();
	private final Shooter shooter = Shooter.getInstance();
	private final Intake intake = Intake.getInstance();
	private final Climber climber = Climber.getInstance();
	private final Music music = Music.getInstance();
	private final Lighting lighting = Lighting.getInstance();

	private final Telemetry logger;
	public Field2d field;

	public void configureDefaultBinds() {
		removeDefaultCommands();

		climber.setDefaultCommand(
				new ClimberAdjust(() -> -xManip.getLeftTriggerAxis(), () -> -xManip.getRightTriggerAxis()));

		if (Robot.isSimulation()) {
			drivetrain
					.setDefaultCommand(
							new xDrive(() -> -simController.getRawAxis(0), () -> -simController.getRawAxis(1),
									() -> simController.getRawAxis(2), () -> 0d));
		} else {
			drivetrain.setDefaultCommand(
					new xDrive(xDrive::getLeftY, xDrive::getLeftX, xDrive::getRightX,
							xDrive::getRightTriggerAxis).ignoringDisable(true));
		}
	}

	public void removeDefaultCommands() {
		Intake.getInstance().removeDefaultCommand();
		Shooter.getInstance().removeDefaultCommand();
		Arm.getInstance().removeDefaultCommand();
		Drivetrain.getInstance().removeDefaultCommand();
		Climber.getInstance().removeDefaultCommand();
		Lighting.getInstance().removeDefaultCommand();
	}

	private void configureDriverBinds() {
		new Trigger(() -> Timer.getMatchTime() > 15).and(() -> Timer.getMatchTime() < 20)
				.whileTrue(new RunCommand(() -> {
					xDrive.getHID().setRumble(RumbleType.kBothRumble, 1);
				})).negate().onTrue(new RunCommand(() -> {
					xDrive.getHID().setRumble(RumbleType.kBothRumble, 0);
				}));

		new Trigger(() -> Math.abs(xDrive.getLeftTriggerAxis()) > Constants.OperatorConstants.Driver.kDeadzone)
				.whileTrue(
						new ClimberAdjust(() -> -xDrive.getLeftTriggerAxis(), () -> -xDrive.getLeftTriggerAxis())
								.alongWith(Lighting.getStrobeCommand(() -> LEDState.kPurple))
								.finallyDo(Lighting.startTimer));

		xDrive.a().toggleOnTrue(new AlignToSpeaker(xDrive::getLeftY, xDrive::getLeftX,
				xDrive::getRightTriggerAxis));

		xDrive.b().toggleOnTrue(new AlignToAmp(xDrive::getLeftY, xDrive::getLeftX,
				xDrive::getRightTriggerAxis));

		xDrive.x().toggleOnTrue(ShootCommandFactory.getAimAndShootCommand());

		xDrive.y().toggleOnTrue(new PathToPoint(Constants.AutoConstants.WayPoints.Blue.kAmp)
				.alongWith(Lighting.getStrobeCommand(() -> LEDState.kGreen))
				.andThen(ShootCommandFactory.getAmpCommand())
				// .alongWith(new DeployAndIntake(true))
				.withName("Auto-pilot Source Intake"));

		// just runs feeder
		xDrive.leftStick()
				.toggleOnTrue(new DeployAndIntake(false).unless(() -> Intake.getInstance().getShooterSensor())
						// .andThen(new BeltDrive(() -> -0.2).withTimeout(1)
						.andThen(Lighting.getStrobeCommand(() -> LEDState.kPink))
						.andThen(new RunCommand(() -> xDrive.getHID().setRumble(RumbleType.kBothRumble,
								Constants.OperatorConstants.Driver.kIntakeRumbleStrength))
								.withTimeout(2)
								.finallyDo(() -> xDrive.getHID().setRumble(RumbleType.kBothRumble, 0)))
						.finallyDo(Lighting.startTimer));

		// deploys intake (right paddle)
		xDrive.rightStick()
				.toggleOnTrue(new DeployAndIntake(true).unless(() -> Intake.getInstance().getShooterSensor())
						// .andThen(new BeltDrive(() -> -0.2).withTimeout(1)
						.andThen(Lighting.getStrobeCommand(() -> LEDState.kPink))
						.andThen(new RunCommand(() -> xDrive.getHID().setRumble(RumbleType.kBothRumble,
								Constants.OperatorConstants.Driver.kIntakeRumbleStrength))
								.withTimeout(2)
								.finallyDo(() -> xDrive.getHID().setRumble(RumbleType.kBothRumble, 0)))
						.finallyDo(Lighting.startTimer));

		xDrive.povLeft().onTrue(new ReturnHome().alongWith(new CancelShooter()));
		xDrive.povDown().onTrue(drivetrain.runOnce(drivetrain::seedFieldRelative).withName("Seed Field Relative"));
		xDrive.povUp().toggleOnTrue(new MultiIntake().alongWith(new Feed(),
				Lighting.getStrobeCommand(() -> LEDState.kPurple)).finallyDo(Lighting.startTimer));

		// Square up to the speaker and press this to reset odometry to the speaker
		xDrive.povRight().onTrue(drivetrain
				.runOnce(() -> drivetrain
						.seedFieldRelative(!Robot.isRed() ? Constants.AutoConstants.WayPoints.Blue.CenterStartPosition
								: GeometryUtil
										.flipFieldPose(Constants.AutoConstants.WayPoints.Blue.CenterStartPosition)))
				.withName("Zero Swerve 2 Speaker"));

		xDrive.leftBumper().toggleOnTrue(ShootCommandFactory.getPrepareAndShootCommand());

		xDrive.rightBumper().toggleOnTrue(new Extend()
				.alongWith(Lighting.getStrobeCommand(() -> LEDState.kWhite))
				.finallyDo(Lighting.startTimer));
	}

	private void configureManipBinds() {
		new Trigger(() -> Timer.getMatchTime() > 15).and(() -> Timer.getMatchTime() < 20)
				.whileTrue(new RunCommand(() -> {
					xManip.getHID().setRumble(RumbleType.kBothRumble, 1);
				})).negate().onTrue(new RunCommand(() -> {
					xManip.getHID().setRumble(RumbleType.kBothRumble, 0);
				}));

		new Trigger(() -> Math.abs(xManip.getRightY()) > Constants.OperatorConstants.Manip.kDeadzone)
				.whileTrue(new FineAdjust(() -> -xManip.getRightY()));

		new Trigger(() -> Math.abs(xManip.getLeftY()) > Constants.OperatorConstants.Manip.kDeadzone)
				.whileTrue(new BeltDrive(() -> -xManip.getLeftY()));

		xManip.x().onTrue(new ReturnHome().alongWith(new CancelShooter()));
		xManip.y().onTrue(HomeClimber.getHomingCommand());
		xManip.a().toggleOnTrue(ShootCommandFactory.getAimAndShootCommandWithWaitUntil(xManip.leftBumper()));
		xManip.b().toggleOnTrue(ShootCommandFactory.getAmpCommandWithWaitUntil(xManip.leftBumper()));

		xManip.rightStick().toggleOnTrue(new Extend().alongWith(Lighting.getStrobeCommand(() -> LEDState.kWhite))
				.finallyDo(Lighting.startTimer));

		xManip.leftStick()
				.toggleOnTrue(new Retract()
						.alongWith(Lighting.getStrobeCommand(() -> Robot.isRed() ? LEDState.kRed : LEDState.kBlue))
						.finallyDo(Lighting.startTimer));

		xManip.povUp().toggleOnTrue(ShootCommandFactory.getPrepareAndShootCommand());
		xManip.povRight().toggleOnTrue(ShootCommandFactory.getRapidFireCommandWithWaitUntil(xManip.leftBumper()));
		xManip.povDown().whileTrue(new Spit());

		xManip.povLeft()
				.toggleOnTrue(new PassToShooter().andThen(
						new SetPoint(Constants.ArmConstants.SetPoints.kCenterToWingPass).deadlineWith(new Prepare()),
						new WaitUntilCommand(() -> xManip.getHID().getLeftBumper()).deadlineWith(new Prepare()),
						new Shoot(false)).finallyDo(ReturnHome.ReturnHome));

		// left bumper is the universal shoot button
		xManip.rightBumper().whileTrue(ShootCommandFactory.getPrepareAndShootCommandWithWaitUntil(xManip.leftBumper()));

		// absolute worst case scenario
		xManip.start().and(() -> xManip.back().getAsBoolean())
				.onTrue(arm.runOnce(arm::toggleArmMotorLimits));

		// TODO: this wont work
		// xManip.back().toggleOnTrue(new DeployAndIntake(true).unless(() ->
		// Intake.getInstance().getShooterSensor())
		// .andThen(Lighting.getStrobeCommand(() -> LEDState.kPink),
		// new RunCommand(() -> xDrive.getHID().setRumble(RumbleType.kBothRumble,
		// 1)).withTimeout(2))
		// .onlyIf(() -> Intake.getInstance().getShooterSensor())
		// .finallyDo(Lighting.startTimer));

		xManip.start().onTrue(new ReturnHome().alongWith(new CancelShooter()));

		xManip.back()
				.toggleOnTrue(new DeployAndIntake(true).unless(() -> Intake.getInstance().getShooterSensor())
						// .andThen(new BeltDrive(() -> -0.2).withTimeout(1)
						.andThen(Lighting.getStrobeCommand(() -> LEDState.kPink))
						.andThen(new RunCommand(() -> xDrive.getHID().setRumble(RumbleType.kBothRumble, 1))
								.withTimeout(2))
						.onlyIf(() -> Intake.getInstance().getShooterSensor())
						.finallyDo(Lighting.startTimer));

		xManip.start().onTrue(new ReturnHome().alongWith(new CancelShooter()));
	}

	public void configurePitBinds() {
		// works perfectly
		xPit.a().toggleOnTrue(ShootCommandFactory.getAimAndShootCommand());

		// works perfectly
		xPit.b().toggleOnTrue(ShootCommandFactory.getAimAndShootCommandWithTimeouts());

		// works perfectly
		xPit.y().toggleOnTrue(ShootCommandFactory.getAimAndShootCommandWithWaitUntil(xPit.leftBumper()));

		// works perfectly
		xPit.x().toggleOnTrue(ShootCommandFactory.getPrepareAndShootCommand());

		// works perfectly
		xPit.povUp().toggleOnTrue(ShootCommandFactory.getAmpCommand());

		// works perfectly
		xPit.povDown().toggleOnTrue(ShootCommandFactory.getAmpCommandWithWaitUntil(xPit.leftBumper()));

		// works perfectly but doesnt return to home or cancel shooter
		xPit.povLeft().toggleOnTrue(ShootCommandFactory.getCenterToWingCommand(xPit.leftBumper()));

		// works perfectly
		xPit.povRight().toggleOnTrue(ShootCommandFactory.getRapidFireCommandWithWaitUntil(xPit.leftBumper()));

		// works perfectly
		xPit.rightBumper().whileTrue(ShootCommandFactory.getPrepareAndShootCommandWithWaitUntil(xPit.leftBumper()));

		// works perfectly
		xPit.rightStick().toggleOnTrue(ShootCommandFactory.getPrepareAndShootCommandWithTimeouts());

		// shoots fine but never cancels the shooter
		xPit.leftStick().toggleOnTrue(ShootCommandFactory.getRapidFireCommand());
	}

	private void configureSimBinds() {
		new Trigger(() -> simController.getRawButtonPressed(1))
				.toggleOnTrue(new AlignToSpeaker(() -> -simController.getRawAxis(0), () -> -simController.getRawAxis(1),
						null));

		new Trigger(() -> simController.getRawButtonPressed(2))
				// .onTrue(arm.runOnce(arm::toggleArmMotorLimits));
				.toggleOnTrue(new SetPoint());

		new Trigger(() -> simController.getRawButtonPressed(3))
				.toggleOnTrue(new PathToPoint(Constants.AutoConstants.WayPoints.Blue.kAmp)
						.alongWith(ShootCommandFactory.getAmpCommandWithWaitUntil(xDrive.leftBumper()))
						.until(() -> Math
								.abs(xDrive.getLeftX()) > Constants.OperatorConstants.Driver.kCommandCancelThreshold
								|| Math.abs(
										xDrive.getLeftY()) > Constants.OperatorConstants.Driver.kCommandCancelThreshold
								|| Math.abs(
										xDrive.getRightX()) > Constants.OperatorConstants.Driver.kCommandCancelThreshold)
						.withName("Auto-pilot Amp shot"));

		new Trigger(() -> simController.getRawButtonPressed(4))
				.toggleOnTrue(ShootCommandFactory.getAimAndShootCommand());
	}

	public RobotContainer() {
		logger = new Telemetry();
		field = Robot.teleopField;
		drivetrain.registerTelemetry((telemetry) -> logger.telemeterize(telemetry));
		configureDriverBinds();
		configureManipBinds();
		configureDefaultBinds();

		if (Robot.isSimulation()) {
			configureSimBinds();
		}
	}
}
