package frc.robot;

import java.util.Map;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Alert.AlertType;
import frc.robot.Constants.Lights.LEDState;
import frc.robot.Vision.LimelightHelpers;
import frc.robot.commands.ShootCommandFactory;
import frc.robot.commands.ArmCommands.SetPoint;
import frc.robot.commands.ArmCommands.TrackSetPoint;
import frc.robot.commands.AutoCommands.PathPlannerCommand;
import frc.robot.commands.ClimberCommands.Extend;
import frc.robot.commands.ClimberCommands.HomeClimber;
import frc.robot.commands.ClimberCommands.Retract;
import frc.robot.commands.IntakeCommands.InfiniteIntake;
import frc.robot.commands.IntakeCommands.DeployAndIntake;
import frc.robot.commands.ShooterCommands.FullSend;
import frc.robot.commands.ShooterCommands.Prepare;
import frc.robot.commands.Swerve.AlignToSpeaker;
import frc.robot.commands.Swerve.DriveToNote;
import frc.robot.commands.Swerve.PathToPoint;
import frc.robot.subsystems.Arm;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.subsystems.Drivetrain.SwerveRequest;

public class Robot extends TimedRobot {
	public static SendableChooser<String> startingPose = new SendableChooser<>();
	public static SendableChooser<String> autoChooser = new SendableChooser<>();
	public static SendableChooser<Pose2d> pathToPointCommandChooser = new SendableChooser<>();
	public static SendableChooser<Boolean> autoShoot = new SendableChooser<>();

	public static SendableChooser<Command> swerveCommands = new SendableChooser<>();
	public static SendableChooser<Command> armCommands = new SendableChooser<>();
	public static final double swerveTestSpeed = 0.1;

	public static SendableChooser<String> songChooser = new SendableChooser<>();
	public static SendableChooser<Boolean> compressChooser = new SendableChooser<>();

	public static NetworkTableEntry pathDelayEntry, desiredSetpointEntry;

	public static ShuffleboardTab teleopTab, autoTab, disabledTab, testTab;

	/** Use teleopField for everything */
	public static Field2d teleopField = new Field2d(), autoField = new Field2d();
	public static PowerDistribution pdh = new PowerDistribution();
	public static Timer timer = new Timer();
	public static RobotContainer robotContainer;

	public static boolean atComp = false;

	public static SequentialCommandGroup autonCommand;
	public static Command pathPlannerCommand;

	public static Command setpointCommand;

	public static Alert forwardAuto, wrongAlliance, StartingPosition;

	@Override
	public void robotInit() {
		forwardAuto = new Alert("Robot will travel forward", Alert.AlertType.INFO);
		wrongAlliance = new Alert("Switch to Blue alliance for best results", Alert.AlertType.INFO);
		StartingPosition = new Alert("Starting Position Undefined", Alert.AlertType.INFO);

		teleopTab = Shuffleboard.getTab("Teleoperated");
		autoTab = Shuffleboard.getTab("Autonomous");
		disabledTab = Shuffleboard.getTab("Disabled");
		testTab = Shuffleboard.getTab("Test");

		robotContainer = new RobotContainer();

		Drivetrain.getInstance().getDaqThread().setThreadPriority(99);

		SmartDashboard.putData("Field", teleopField);

		songChooser.setDefaultOption("The defult option is nothing", "");
		songChooser.addOption("Imperial March", "Sith.chrp");
		songChooser.addOption("Under Pressure ", "underpressure.chrp");
		songChooser.addOption("Sweet Caroline", "sweetcaroline.chrp");
		songChooser.addOption("Another One Bites The Dust ", "anotheronebitesthedust.chrp");

		startingPose.addOption("Amp Side", "amp"); // left
		startingPose.setDefaultOption("Center", "center"); // center
		startingPose.addOption("Stage Side", "stage"); // right
		startingPose.addOption("Out West", "out west"); // right

		autoChooser.setDefaultOption("Sit and Shit", "null");
		autoChooser.addOption("Simple Reverse", "back-up");
		autoChooser.addOption("All Close Notes", " all close notes");
		autoChooser.addOption("All Close Notes Plus far Piece", " all close notes plus");
		autoChooser.addOption("Close 2 Piece (No Center)", " close 2 piece");
		autoChooser.addOption("1 Piece", " 1 piece");
		autoChooser.addOption("2 Piece", " 2 piece");
		autoChooser.addOption("3 Piece", " 3 piece");
		autoChooser.addOption("4 Piece (Center Only)", " 4 piece");
		autoChooser.addOption("Far 1 Piece (No Center)", " far 1 piece");
		autoChooser.addOption("Far 2 Piece (No Center)", " far 2 piece");
		autoChooser.addOption("Far 3 Piece (No Center)", " far 3 piece");
		autoChooser.addOption("Devious Auto (Stage Only)", " devious lick");

		autoShoot.setDefaultOption("Dont Shoot At Start", false);
		autoShoot.addOption("Shoot At Start", true);

		pathToPointCommandChooser.setDefaultOption("None", null);
		pathToPointCommandChooser.addOption("Far Amp Note",
				Constants.AutoConstants.WayPoints.CenterNotes.farLeft);
		pathToPointCommandChooser.addOption("Far Mid Amp Note",
				Constants.AutoConstants.WayPoints.CenterNotes.farMidLeft);
		pathToPointCommandChooser.addOption("Far Center Note",
				Constants.AutoConstants.WayPoints.CenterNotes.farCenter);
		pathToPointCommandChooser.addOption("Far Mid Source Note",
				Constants.AutoConstants.WayPoints.CenterNotes.farMidRight);
		pathToPointCommandChooser.addOption("Far Source Note",
				Constants.AutoConstants.WayPoints.CenterNotes.farRight);

		compressChooser.setDefaultOption("Compresor Enabled", true);
		compressChooser.addOption("Compressor Disabled", false);

		autoTab.add("Starting Pose Selector", startingPose)
				.withSize(3, 1)
				.withPosition(0, 3)
				.withWidget(BuiltInWidgets.kSplitButtonChooser);

		autoTab.add("Path Selector", autoChooser)
				.withSize(3, 1)
				.withPosition(0, 2)
				.withWidget(BuiltInWidgets.kComboBoxChooser);

		autoTab.add("Shoot Selector", autoShoot)
				.withSize(3, 1)
				.withPosition(3, 3)
				.withWidget(BuiltInWidgets.kSplitButtonChooser);

		autoTab.add("Center-line Sprint Path to Point Selector", pathToPointCommandChooser)
				.withSize(6, 1)
				.withPosition(0, 4)
				.withWidget(BuiltInWidgets.kSplitButtonChooser);

		autoTab.add("Path Delay", 0)
				.withSize(3, 1)
				.withPosition(3, 2)
				.withWidget(BuiltInWidgets.kNumberSlider)
				.withProperties(Map.of("min_value", 0, "max_value", 15, "block increment", 1, "Divisions", 6));
		// .withProperties(Map.of("min_value", 0, "min", 0, "max_value", 15, "max", 15,
		// "block increment", 1, "divisions", 6));

		autoTab.addNumber("Auto Countdown", () -> Timer.getMatchTime())
				.withSize(3, 2)
				.withPosition(3, 0)
				.withWidget("Match Time")
				.withProperties(Map.of("red_start_time", -1, "yellow_start_time", -1));

		autoTab.addBoolean("Ready To Shoot", () -> Shooter.getInstance().readyToShootAdvanced())
				.withSize(3, 2)
				.withPosition(0, 0)
				.withWidget(BuiltInWidgets.kBooleanBox);

		teleopTab
				.addBoolean("Ready To Shoot",
						() -> Shooter.getInstance().readyToShootAdvanced() && Drivetrain.getInstance().readyToShoot())
				.withSize(3, 2)
				.withPosition(0, 0);

		teleopTab.addNumber("Match Time", () -> Timer.getMatchTime())
				.withSize(3, 4)
				.withPosition(3, 0)
				.withWidget("Match Time")
				.withProperties(Map.of("red_start_time", 15, "yellow_start_time", 30));

		teleopTab.addBoolean("Holding", () -> Intake.getInstance().getShooterSensor())
				.withSize(3, 2)
				.withPosition(0, 2)
				.withWidget(BuiltInWidgets.kBooleanBox);

		testTab.add("Desired Setpoint", Constants.ArmConstants.SetPoints.kIntake)
				.withSize(3, 1)
				.withPosition(2, 0)
				.withWidget(BuiltInWidgets.kTextView)
				.withProperties(Map.of("show_submit_button", true));

		desiredSetpointEntry = NetworkTableInstance.getDefault()
				.getTable("Shuffleboard")
				.getSubTable(testTab.getTitle())
				.getEntry("Desired Setpoint");

		// use this for shooter regression
		setpointCommand = new TrackSetPoint(
				() -> desiredSetpointEntry.getDouble(Constants.ArmConstants.SetPoints.kIntake));

		testTab.add(setpointCommand)
				.withSize(2, 1)
				.withPosition(5, 0)
				.withWidget(BuiltInWidgets.kCommand);

		pathDelayEntry = NetworkTableInstance.getDefault()
				.getTable("Shuffleboard")
				.getSubTable(autoTab.getTitle())
				.getEntry("Path Delay");

		SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());

		disabledTab.add("Command Scheduler", CommandScheduler.getInstance());

		disabledTab.add("Song Selector", songChooser)
				.withSize(3, 1)
				.withPosition(0, 3)
				.withWidget(BuiltInWidgets.kComboBoxChooser);
				
		disabledTab.add("Toggle Compression", compressChooser)
				.withSize(3, 1)
				.withPosition(0, 4)
				.withWidget(BuiltInWidgets.kSplitButtonChooser);

		Alert.groups.forEach((group, alert) -> {
			disabledTab.add(group, alert)
					.withSize(3, 3)
					.withPosition(0, 0)
					.withWidget("Alerts");

			testTab.add(group, alert)
					.withSize(2, 3)
					.withPosition(0, 0)
					.withWidget("Alerts");
		});

		// drive forward command
		swerveCommands.setDefaultOption("Do Nothing (Reset Gyro)",
				Drivetrain.getInstance()
						.runOnce(() -> Drivetrain.getInstance()
								.seedFieldRelative(!Robot.isRed()
										? Constants.AutoConstants.WayPoints.Blue.CenterStartPosition
										: GeometryUtil
												.flipFieldPose(
														Constants.AutoConstants.WayPoints.Blue.CenterStartPosition)))
						.withName("Zero Swerve 2 Speaker"));

		swerveCommands.addOption("Drive Forward",
				Drivetrain.getInstance().applyRequest(() -> new SwerveRequest.FieldCentric()
						.withVelocityX(
								Constants.SwerveConstants.SwerveSpeeds.kMaxSpeedMetersPerSecond * swerveTestSpeed))
						.withName("Drive Forward"));

		swerveCommands.addOption("Drive Backwards",
				Drivetrain.getInstance().applyRequest(() -> new SwerveRequest.FieldCentric()
						.withVelocityX(
								-Constants.SwerveConstants.SwerveSpeeds.kMaxSpeedMetersPerSecond * swerveTestSpeed))
						.withName("Drive Backwards"));

		swerveCommands.addOption("Drive Left",
				Drivetrain.getInstance().applyRequest(() -> new SwerveRequest.FieldCentric()
						.withVelocityY(
								Constants.SwerveConstants.SwerveSpeeds.kMaxSpeedMetersPerSecond * swerveTestSpeed))
						.withName("Drive Left"));

		swerveCommands.addOption("Drive Right",
				Drivetrain.getInstance().applyRequest(() -> new SwerveRequest.FieldCentric()
						.withVelocityY(
								-Constants.SwerveConstants.SwerveSpeeds.kMaxSpeedMetersPerSecond * swerveTestSpeed))
						.withName("Drive Right"));

		swerveCommands.addOption("Rotate",
				Drivetrain.getInstance().applyRequest(() -> new SwerveRequest.FieldCentric()
						.withRotationalRate(
								Constants.SwerveConstants.SwerveSpeeds.kMaxAngularSpeedRadiansPerSecond
										* swerveTestSpeed))
						.withName("Rotate"));

		testTab.add("Swerve Commands", swerveCommands)
				.withSize(9, 1)
				.withPosition(2, 2)
				.withWidget(BuiltInWidgets.kSplitButtonChooser);

		armCommands.setDefaultOption("Nothing", new InstantCommand());
		armCommands.addOption("Intake", new SetPoint(Constants.ArmConstants.SetPoints.kIntake));
		armCommands.addOption("Track Speaker", new SetPoint());
		armCommands.addOption("0 degrees", new SetPoint(0));
		armCommands.addOption("Amp", new SetPoint(Constants.ArmConstants.SetPoints.kAmp));

		testTab.add("Arm Commands", armCommands)
				.withSize(5, 1)
				.withPosition(2, 1)
				.withWidget(BuiltInWidgets.kSplitButtonChooser);

		testTab.add("Extend Climber", new Extend())
				.withSize(2, 1)
				.withPosition(9, 0)
				.withWidget(BuiltInWidgets.kCommand);

		testTab.add("Retract Climber", new Retract())
				.withSize(2, 1)
				.withPosition(9, 1)
				.withWidget(BuiltInWidgets.kCommand);

		testTab.add("Intake (with deploying)", new DeployAndIntake(true))
				.withSize(2, 1)
				.withPosition(7, 0)
				.withWidget(BuiltInWidgets.kCommand);

		testTab.add("Intake (without deploying)", new DeployAndIntake(false))
				.withSize(2, 1)
				.withPosition(7, 1)
				.withWidget(BuiltInWidgets.kCommand);

		if (Robot.isReal() && Constants.Vision.UseLimelight) {
			// front camera (intake cam) - auto tab
			autoTab.add(new HttpCamera(Constants.Vision.LimelightPython.llPython,
					Constants.Vision.LimelightPython.llPythonIP))
					.withSize(9, 7)
					.withPosition(6, 0)
					.withWidget(BuiltInWidgets.kCameraStream)
					.withProperties(Map.of("Show Crosshair", false, "Show Controls", false));

			// front camera (intake cam) - teleop tab
			teleopTab
					.add(new HttpCamera(Constants.Vision.LimelightPython.llPython,
							Constants.Vision.LimelightPython.llPythonIP))
					.withSize(9, 7)
					.withPosition(6, 0)
					.withWidget(BuiltInWidgets.kCameraStream)
					.withProperties(Map.of("Show Crosshair", false, "Show Controls", false));

			LimelightHelpers.setPipelineIndex(Constants.Vision.LimelightFront.llAprilTag,
					Constants.Vision.LimelightFront.llAprilTagPipelineIndex);
			LimelightHelpers.setPipelineIndex(Constants.Vision.LimelightRear.llAprilTagRear,
					Constants.Vision.LimelightFront.llAprilTagPipelineIndex);
			LimelightHelpers.setPipelineIndex(Constants.Vision.LimelightPython.llPython,
					Constants.Vision.LimelightPython.llPythonPipelineIndex);

			// check if cameras are all connected
			disabledTab.add(new HttpCamera(Constants.Vision.LimelightFront.llAprilTag, Constants.Vision.LimelightFront.llAprilTagIP))
					.withSize(2, 2)
					.withPosition(3, 0)
					.withWidget(BuiltInWidgets.kCameraStream)
					.withProperties(Map.of("Show Crosshair", false, "Show Controls", false));

			disabledTab.add(new HttpCamera(Constants.Vision.LimelightRear.llAprilTagRear, Constants.Vision.LimelightRear.llAprilTagRearIP))
					.withSize(2, 2)
					.withPosition(5, 0)
					.withWidget(BuiltInWidgets.kCameraStream)
					.withProperties(Map.of("Show Crosshair", false, "Show Controls", false));

			disabledTab.add(new HttpCamera(Constants.Vision.LimelightPython.llPython, Constants.Vision.LimelightPython.llPythonIP))
					.withSize(2, 2)
					.withPosition(7, 0)
					.withWidget(BuiltInWidgets.kCameraStream)
					.withProperties(Map.of("Show Crosshair", false, "Show Controls", false));

			// check if cameras are working
			testTab.add(new HttpCamera(Constants.Vision.LimelightFront.llAprilTag, Constants.Vision.LimelightFront.llAprilTagIP))
					.withSize(3, 2)
					.withPosition(1, 3)
					.withWidget(BuiltInWidgets.kCameraStream)
					.withProperties(Map.of("Show Crosshair", false, "Show Controls", false));

			testTab.add(new HttpCamera(Constants.Vision.LimelightRear.llAprilTagRear, Constants.Vision.LimelightRear.llAprilTagRearIP))
					.withSize(3, 2)
					.withPosition(4, 3)
					.withWidget(BuiltInWidgets.kCameraStream)
					.withProperties(Map.of("Show Crosshair", false, "Show Controls", false));

			testTab.add(new HttpCamera(Constants.Vision.LimelightPython.llPython, Constants.Vision.LimelightPython.llPythonIP))
					.withSize(3, 2)
					.withPosition(7, 3)
					.withWidget(BuiltInWidgets.kCameraStream)
					.withProperties(Map.of("Show Crosshair", false, "Show Controls", false));
		}

		DriverStation.silenceJoystickConnectionWarning(true);

		Shuffleboard.selectTab(disabledTab.getTitle());

		Lighting.getInstance().autoSetLights(true);

		NamedCommands.registerCommand("Shoot",
				ShootCommandFactory.getAimAndShootCommandWithTimeouts()
						.deadlineWith(new AlignToSpeaker(),
								new InstantCommand(() -> Lighting.getInstance().autoSetLights(true)))
						.withName("Align and Shoot"));

		NamedCommands.registerCommand("Prepare", new SetPoint().alongWith(new Prepare()));
		NamedCommands.registerCommand("Intake", new DeployAndIntake(true));
		NamedCommands.registerCommand("Shoot on the fly", ShootCommandFactory.getAimAndShootCommandWithTimeouts());
		NamedCommands.registerCommand("Lower Arm",
				// doing this so that we dont have to wait for arm velocity to be 0, and as soon
				// as it is within 4 degrees of the setpoint then just end this command and
				// follow path
				new SetPoint(Constants.ArmConstants.kArmMinAngle).raceWith(new WaitUntilCommand(
						() -> Arm.getInstance().isInRangeOfTarget(Constants.ArmConstants.kArmMinAngle, 4)))
						.withName("Lowering arm to hard stop"));

		NamedCommands.registerCommand("Auto Intake",
				new DeployAndIntake(true).raceWith(new DriveToNote().onlyWhile(
						// dont go over the halfway line
						() -> Drivetrain.getInstance().getPose().getX() + (0.92 / 2) < Constants.fieldLength / 2 - 0.5))
						.onlyIf(DriveToNote.thereIsANote)
						.withName("Auto Intake")
						.withTimeout(2));

		NamedCommands.registerCommand("Devious Shooting",
				new InfiniteIntake(0.2)
						.alongWith(new FullSend(0.2), Lighting.getLarsonCommand(() -> LEDState.kWhite))
						.finallyDo(Lighting.startTimer)
						.withName("Devious Shooting"));

	}

	@Override
	public void driverStationConnected() {
		if (DriverStation.isFMSAttached()) {
			atComp = true;
			Shuffleboard.selectTab(autoTab.getTitle());
		}

		// other than if a motor or cancoder fails to verify (position isnt available),
		// only once driverstation connects should we start logging stuff
		DataLogManager.start(Constants.logDirectory);
		DataLogManager.log("Driverstation connected");
		DriverStation.startDataLog(DataLogManager.getLog());

		CommandScheduler.getInstance()
				.onCommandInitialize((action) -> DataLogManager.log(action.getName() + " Command Initialized"));
		CommandScheduler.getInstance()
				.onCommandInterrupt((action) -> DataLogManager.log(action.getName() + " Command Interrupted"));
		CommandScheduler.getInstance()
				.onCommandFinish((action) -> DataLogManager.log(action.getName() + " Command Finished"));
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();

	}

	@Override
	public void disabledInit() {
		Shuffleboard.selectTab(disabledTab.getTitle());
		LimelightHelpers.setLEDMode_ForceOff(Constants.Vision.LimelightPython.llPython);
	}

	@Override
	public void disabledPeriodic() {
		PathPlannerCommand.publishTrajectory(startingPose.getSelected() + autoChooser.getSelected());
		try {
			Pose2d centerPose = pathToPointCommandChooser.getSelected();
			teleopField.getObject("Last Ditch Effort").setPose(centerPose);
		} catch (Exception e) {
			teleopField.getObject("Last Ditch Effort").setPose(new Pose2d(-5, -5, Rotation2d.fromDegrees(0)));
		}
	}

	@Override
	public void disabledExit() {
		LimelightHelpers.setLEDMode_ForceOn(Constants.Vision.LimelightPython.llPython);
	}

	@Override
	public void autonomousInit() {
		Arm.getInstance().setBrake(true);
		Shooter.getInstance().setShooterBrake(false);
		Arm.getInstance().setMotionMagic(Constants.ArmConstants.SetPoints.kIntake);

		Shuffleboard.selectTab(autoTab.getTitle());
		robotContainer.removeDefaultCommands();

		try {
			pathPlannerCommand = AutoBuilder.buildAuto(startingPose.getSelected() + autoChooser.getSelected());
		} catch (RuntimeException e) {
			if (autoChooser.getSelected().equals("back-up") || autoChooser.getSelected().equals("null")) {
				if (autoChooser.getSelected().equals("back-up")) {
					pathPlannerCommand = Drivetrain.getInstance()
							.applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(0.75));
				}

				Pose2d startPose;

				switch (startingPose.getSelected()) {
					case "amp":
						startPose = Constants.AutoConstants.WayPoints.Blue.AmpStartPosition;
						StartingPosition.set(false);
						break;
					case "center":
						startPose = Constants.AutoConstants.WayPoints.Blue.CenterStartPosition;
						StartingPosition.set(false);
						break;
					case "stage":
						startPose = Constants.AutoConstants.WayPoints.Blue.StageStartPosition;
						StartingPosition.set(false);
						break;
					default:
						// just dont seed the pose, instead set it to be the robot pose
						StartingPosition.set(true);
						System.out.println("Starting Position Undefined");
						startPose = Drivetrain.getInstance().getPose();
						break;
				}

				if (Robot.isRed()) {
					startPose = GeometryUtil.flipFieldPose(startPose);
				}

				Drivetrain.getInstance().seedFieldRelative(startPose);

				forwardAuto.set(true);
				DataLogManager.log("Backing Up");
			}

			else {
				pathPlannerCommand = new PrintCommand(
						"Null Path: " + startingPose.getSelected() + autoChooser.getSelected());
				DataLogManager.log("Autonomous init: Invalid Auto");
			}
		} catch (Exception e) {
			DataLogManager.log("Auto not working actual problem. Stacktrace:\n" + e.getStackTrace());
			pathPlannerCommand = new PrintCommand("Autobuilder Exception");
			e.printStackTrace();
		}

		Arm.getInstance().setMotionMagic(Constants.ArmConstants.SetPoints.kIntake);

		autonCommand = new SequentialCommandGroup(
				new InstantCommand(timer::restart),
				ShootCommandFactory.getPrepareAndShootCommandWithTimeouts().onlyIf(autoShoot::getSelected),
				new WaitUntilCommand(() -> timer.get() > pathDelayEntry.getDouble(0)),
				pathPlannerCommand,
				new InstantCommand(timer::stop));

		if (pathToPointCommandChooser.getSelected() != null) {
			autonCommand.addCommands(
					new PathToPoint(pathToPointCommandChooser.getSelected()).alongWith(new DeployAndIntake(true)
							.andThen(Lighting.getStrobeCommand(() -> LEDState.kRed)).finallyDo(Lighting.startTimer)));
			Drivetrain.getInstance().getField().getObject("Last Ditch Effort")
					.setPose(pathToPointCommandChooser.getSelected());

		}

		autonCommand.withName("Auto Command").schedule();
		HomeClimber.getHomingCommand().schedule();
	}

	@Override
	public void autonomousPeriodic() {
		NetworkTableInstance.getDefault().getTable("PathPlanner").getEntry("Auto Timer").setDouble(timer.get());
	}

	@Override
	public void autonomousExit() {
		timer.stop();
		Drivetrain.getInstance().getField().getObject("Last Ditch Effort")
				.setPose(new Pose2d(-5, -5, Rotation2d.fromDegrees(0)));
	}

	@Override
	public void teleopInit() {
		LimelightHelpers.setLEDMode_ForceOn(Constants.Vision.LimelightPython.llPython);

		robotContainer.configureDefaultBinds();

		Arm.getInstance().setBrake(true);
		Arm.getInstance().setMotionMagic(Constants.ArmConstants.SetPoints.kIntake);

		Shooter.getInstance().setShooterBrake(true);

		PathPlannerCommand.unpublishTrajectory();

		Shuffleboard.selectTab(teleopTab.getTitle());
		HomeClimber.getHomingCommand().schedule();
	}

	@Override
	public void teleopPeriodic() {

	}

	@Override
	public void teleopExit() {
		Lighting.getLarsonCommand(() -> LEDState.kRobostangsOrange).schedule();
	}

	Command lastSwerve, lastArm;

	@Override
	public void testInit() {
		if (!compressChooser.getSelected()) {
			Intake.getInstance().disableCompressor();
		}

		robotContainer.configurePitBinds();

		CommandScheduler.getInstance().cancelAll();
		Shuffleboard.selectTab(testTab.getTitle());
		HomeClimber.getHomingCommand().schedule();

		lastSwerve = swerveCommands.getSelected();
		lastArm = armCommands.getSelected();
		lastSwerve.schedule();
		lastArm.schedule();

		Lighting.getStrobeCommand(() -> LEDState.kWhite).schedule();
	}

	@Override
	public void testPeriodic() {
		if (swerveCommands.getSelected() != lastSwerve) {
			swerveCommands.getSelected().schedule();
		}

		if (armCommands.getSelected() != lastArm) {
			armCommands.getSelected().schedule();
		}

		wrongAlliance.set(Robot.isRed());

		lastSwerve = swerveCommands.getSelected();
		lastArm = armCommands.getSelected();
		// if(!songChooser.equals("")){
		// Music.getInstance().playMusic(songChooser.getSelected());
		// }
	}

	@Override
	public void testExit() {
		wrongAlliance.set(false);
		Lighting.getInstance().autoSetLights(true);
		Intake.getInstance().enableCompressor();
	}

	@Override
	public void simulationPeriodic() {
	}

	@Override
	public void simulationInit() {
	}

	/**
	 * Returns the alliance color of the robot
	 * 
	 * @return will only return true if the robot is set to red, otherwise it
	 *         returns false (blue)
	 */
	public static boolean isRed() {
		var alliance = DriverStation.getAlliance();

		if (alliance.isEmpty()) {
			return false;
		} else if (alliance.get() == DriverStation.Alliance.Red) {
			return true;
		} else {
			return false;
		}
	}

	public static void verifyMotors(TalonFX... falcons) {
		for (TalonFX falcon : falcons) {
			verifyMotor(falcon);
		}
	}

	/**
	 * Will return false if the motor is verified and connected, true if there is
	 * some error getting position
	 * 
	 * @param falcon a TalonFX motor
	 * @return false if the position is available, true if not available
	 */
	public static boolean verifyMotor(TalonFX falcon) {
		falcon.getConfigurator().apply(new AudioConfigs().withAllowMusicDurDisable(true));

		StatusCode status = falcon.getPosition().getStatus();
		if (status.isError() && Robot.isReal()) {
			DataLogManager.log("TalonFX ID #" + falcon.getDeviceID() + " has failed to return position with status: "
					+ status.getDescription() + ". Error Code: " + status.value);
			new Alert(
					"TalonFX ID #" + falcon.getDeviceID() + " has failed to return position with status: "
							+ status.getName() + ". Error Code: " + status.value,
					AlertType.ERROR).set(true);
			return true;
		}

		return false;
	}

	/**
	 * Will return false if the CANcoder is verified and connected, true if there is
	 * some error getting position
	 * 
	 * @param coder a CANcoder
	 * @return false if the position is available, true if not available
	 */
	public static boolean verifyCANcoder(CANcoder coder) {
		StatusCode status = coder.getPosition().getStatus();
		if (status.isError() && Robot.isReal()) {
			DataLogManager.log("CANcoder ID #" + coder.getDeviceID() + " has failed to return position with status: "
					+ status.getDescription() + ". Error Code: " + status.value);
			new Alert(
					"CANcoder ID #" + coder.getDeviceID() + " has failed to return position with status: "
							+ status.getName() + ". Error Code: " + status.value,
					AlertType.ERROR).set(true);
			return true;
		}

		return false;
	}
}