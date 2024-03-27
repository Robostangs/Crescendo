package frc.robot;

import java.util.Map;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cscore.HttpCamera;
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
import frc.robot.commands.ClimberCommands.HomeClimber;
import frc.robot.commands.IntakeCommands.DeployAndIntake;
import frc.robot.commands.Swerve.Align;
import frc.robot.subsystems.Arm;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.subsystems.Drivetrain.SwerveRequest;

public class Robot extends TimedRobot {
	public static SendableChooser<String> startingPose = new SendableChooser<>();
	public static SendableChooser<String> autoChooser = new SendableChooser<>();
	public static SendableChooser<Boolean> autoShoot = new SendableChooser<>();
	public static NetworkTableEntry pathDelayEntry;

	public static ShuffleboardTab autoTab, teleopTab, disabledTab;

	/** Use teleopField for everything */
	public static Field2d teleopField = new Field2d(), autoField = new Field2d();
	public static PowerDistribution pdh = new PowerDistribution();
	public static Timer timer = new Timer();
	public static RobotContainer robotContainer;

	public static boolean atComp = false;
	public String status = "";

	public static SequentialCommandGroup autonCommand;
	public static Command pathPlannerCommand;

	public static Command setpointCommand;

	public static Alert ShuffleBoardCamera = new Alert("Failed to add camera to Shuffleboard",
			Alert.AlertType.ERROR);
	public static Alert forwardAuto = new Alert("Robot will travel forward", Alert.AlertType.INFO);

	@Override
	public void robotInit() {
		DataLogManager.start(Constants.logDirectory);
		DriverStation.startDataLog(DataLogManager.getLog());

		CommandScheduler.getInstance()
				.onCommandInitialize((action) -> DataLogManager.log(action.getName() + " Command Initialized"));
		CommandScheduler.getInstance()
				.onCommandInterrupt((action) -> DataLogManager.log(action.getName() + " Command Interrupted"));
		CommandScheduler.getInstance()
				.onCommandFinish((action) -> DataLogManager.log(action.getName() + " Command Finished"));

		autoTab = Shuffleboard.getTab("Autonomous");
		teleopTab = Shuffleboard.getTab("Teleoperated");
		disabledTab = Shuffleboard.getTab("Disabled");

		robotContainer = new RobotContainer();

		Drivetrain.getInstance().getDaqThread().setThreadPriority(99);

		SmartDashboard.putData("Field", teleopField);

		startingPose.setDefaultOption("Center", "center"); // center
		startingPose.addOption("Amp Side", "amp"); // left
		startingPose.addOption("Stage Side", "stage"); // right

		autoChooser.setDefaultOption("Sit and Shit", " null");
		autoChooser.addOption("Simple Reverse", "back-up");
		autoChooser.addOption("1 Piece", " 1 piece");
		autoChooser.addOption("2 Piece", " 2 piece");
		autoChooser.addOption("3 Piece", " 3 piece");
		autoChooser.addOption("4 Piece (Center Only)", " 4 piece");
		autoChooser.addOption("All Close Notes (Center Only)", " all close notes");
		autoChooser.addOption("All Close Notes Fast", " all close notes fast");
		autoChooser.addOption("Close 2 Piece (No Center)", " close 2 piece");
		autoChooser.addOption("Far 1 Piece (No Center)", " far 1 piece");
		autoChooser.addOption("Far 2 Piece (No Center)", " far 2 piece");
		autoChooser.addOption("Far 3 Piece", " far 3 piece");

		autoShoot.setDefaultOption("Shoot At Start", true);
		autoShoot.addOption("Dont Shoot At Start", false);

		autoTab.add("Starting Pose Selector", startingPose).withSize(2, 1).withPosition(0, 0)
				.withWidget(BuiltInWidgets.kComboBoxChooser);
		autoTab.add("Path Selector", autoChooser).withSize(2, 1).withPosition(0, 1)
				.withWidget(BuiltInWidgets.kComboBoxChooser);
		autoTab.add("Shoot Selector", autoShoot).withSize(2, 1).withPosition(0, 2)
				.withWidget(BuiltInWidgets.kComboBoxChooser);
		autoTab.addNumber("Auto Countdown", () -> Timer.getMatchTime()).withSize(2, 2).withPosition(2, 1)
				.withWidget("Match Time");
		autoTab.add("Path Delay", 0).withSize(2, 1).withWidget(BuiltInWidgets.kNumberSlider).withPosition(0, 3)
				.withProperties(Map.of("min", 0, "max", 15, "block increment", 1));
		autoTab.addBoolean("Ready To Shoot", () -> Shooter.getInstance().readyToShootAdvanced()).withPosition(2, 0)
				.withSize(2, 1).withWidget(BuiltInWidgets.kBooleanBox);

		autoTab.addString("Auto Status", () -> status).withPosition(2, 3).withSize(2, 1)
				.withWidget(BuiltInWidgets.kTextView);

		teleopTab.addNumber("Match Time", () -> Timer.getMatchTime()).withSize(2, 2).withPosition(2, 0)
				.withWidget("Match Time");

		teleopTab.addBoolean("Holding", () -> Intake.getInstance().getShooterSensor()).withSize(2, 2).withPosition(0, 2)
				.withWidget(BuiltInWidgets.kBooleanBox);
		teleopTab
				.addBoolean("Ready To Shoot",
						() -> Shooter.getInstance().readyToShootAdvanced() && Drivetrain.getInstance().readyToShoot())
				.withSize(2, 2)
				.withPosition(0, 0);
		// use this for shooter regression
		setpointCommand = new TrackSetPoint(
				() -> SmartDashboard.getNumber("Arm/Desired Setpoint", Constants.ArmConstants.SetPoints.kIntake));
		teleopTab.add(setpointCommand).withWidget(BuiltInWidgets.kCommand).withPosition(2, 2).withSize(2, 1);
		teleopTab.add("Desired Setpoint", Constants.ArmConstants.SetPoints.kIntake).withWidget(BuiltInWidgets.kTextView)
				.withPosition(2, 3).withSize(2, 1);

		pathDelayEntry = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable(autoTab.getTitle())
				.getEntry("Path Delay");

		pathDelayEntry = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable(teleopTab.getTitle())
				.getEntry("Desired Setpoint");

		disabledTab.add("Alerts", SmartDashboard.getData("Alerts")).withWidget("Alerts").withSize(3, 3);

		if (Robot.isReal() && Constants.Vision.UseLimelight) {
			try {
				// front camera (intake cam) - auto tab
				autoTab.add(new HttpCamera(Constants.Vision.llPython, Constants.Vision.llPythonIP))
						.withWidget(BuiltInWidgets.kCameraStream).withSize(5, 4).withPosition(4, 0)
						.withProperties(Map.of("Show Crosshair", false, "Show Controls", false));

				// front camera (intake cam) - teleop tab
				teleopTab.add(new HttpCamera(Constants.Vision.llPython, Constants.Vision.llPythonIP))
						.withWidget(BuiltInWidgets.kCameraStream).withSize(10, 8).withPosition(4, 0)
						.withProperties(Map.of("Show Crosshair", false, "Show Controls", false));

				// TODO: should we get rid of rear camera?
				// rear camera (shooting cam) - auto tab
				// autoTab.add(new HttpCamera(Constants.Vision.llAprilTagRear,
				// Constants.Vision.llAprilTagRearIP))
				// .withWidget(BuiltInWidgets.kCameraStream).withSize(5, 4).withPosition(9, 0)
				// .withProperties(Map.of("Show Crosshair", true, "Show Controls", false));

				// rear camera (shooting cam) - teleop tab
				// teleopTab.add(new HttpCamera(Constants.Vision.llAprilTagRear,
				// Constants.Vision.llAprilTagRearIP))
				// .withWidget(BuiltInWidgets.kCameraStream).withSize(5, 4).withPosition(9, 0)
				// .withProperties(Map.of("Show Crosshair", true, "Show Controls", false));
			} catch (Exception e) {
				System.out.println("Failed to add camera to Shuffleboard");
				ShuffleBoardCamera.set(true);
			}
		}

		DriverStation.silenceJoystickConnectionWarning(true);
		Shuffleboard.selectTab(disabledTab.getTitle());

		Lighting.getInstance().autoSetLights(true);

		NamedCommands.registerCommand("Intake", new DeployAndIntake(true));
		NamedCommands.registerCommand("Shoot",
				ShootCommandFactory.getAimAndShootCommandWithTimeouts().deadlineWith(new Align(false)).withName("Align and Shoot"));
		NamedCommands.registerCommand("Lower Arm",
				// doing this so that we dont have to wait for arm velocity to be 0, and as soon
				// as it is under then just end this command and follow path
				new SetPoint(Constants.ArmConstants.kArmMinAngle).raceWith(new WaitUntilCommand(
						// TODO: is 4 degrees too much?
						() -> Arm.getInstance().isInRangeOfTarget(Constants.ArmConstants.kArmMinAngle, 4)))
						// we dont want to put a timeout on this because its important
						// .withTimeout(Constants.OperatorConstants.setpointTimeout)
						.withName("Lowering arm to hard stop"));

		SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
	}

	@Override
	public void driverStationConnected() {
		if (DriverStation.isFMSAttached()) {
			atComp = true;
			Shuffleboard.startRecording();
		}
	}

	@Override
	public void robotPeriodic() {

		CommandScheduler.getInstance().run();
	}

	@Override
	public void disabledInit() {
		Shuffleboard.selectTab(disabledTab.getTitle());
	}

	@Override
	public void disabledPeriodic() {
		PathPlannerCommand.publishTrajectory(startingPose.getSelected() + autoChooser.getSelected());
		autoField.setRobotPose(Drivetrain.getInstance().getPose());
		LimelightHelpers.setLEDMode_ForceOff(Constants.Vision.llPython);
	}

	@Override
	public void disabledExit() {
	}

	@Override
	public void autonomousInit() {
		Arm.getInstance().setBrake(true);
		Shooter.getInstance().setShooterBrake(true);
		Arm.getInstance().setMotionMagic(Constants.ArmConstants.SetPoints.kIntake);

		// TODO: this means it will always track the speaker, however will not allow us
		// to pick up the piece
		// however if, in the paths, we tell the arm to go down then the problem should
		// be solved
		// this should be removed if we cant feed to shooter very well
		// Arm.getInstance().setDefaultCommand(new SetPoint());

		Shuffleboard.selectTab(autoTab.getTitle());
		robotContainer.removeDefaultCommands();

		try {
			pathPlannerCommand = AutoBuilder.buildAuto(startingPose.getSelected() + autoChooser.getSelected());
		} catch (RuntimeException e) {
			if (autoChooser.getSelected().equals("back-up")) {
				pathPlannerCommand = Drivetrain.getInstance()
						.applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(0.5));
				forwardAuto.set(true);
			}

			else {
				pathPlannerCommand = new PrintCommand(
						"Null Path: " + startingPose.getSelected() + autoChooser.getSelected());
				System.out.println("Invalid Auto");
			}
		} catch (Exception e) {
			System.out.println("Auto not working actual problem");
			pathPlannerCommand = new PrintCommand("Autobuilder Exception");
			e.printStackTrace();
		}

		Arm.getInstance().setMotionMagic(Constants.ArmConstants.SetPoints.kIntake);

		autonCommand = new SequentialCommandGroup(
				new InstantCommand(timer::restart),
				ShootCommandFactory.getPrepareAndShootCommandWithTimeouts().onlyIf(autoShoot::getSelected),
				new WaitUntilCommand(pathDelayEntry.getDouble(0)), pathPlannerCommand,
				new InstantCommand(timer::stop));

		// if (autoShoot.getSelected()) {
		// 	// we want prepare and shoot because we know that at the beginning of the match,
		// 	// the robot will start in a position where it is ready to shoot off rip
		// 	autonCommand.beforeStarting(ShootCommandFactory.getPrepareAndShootCommand());
		// }

		if (Constants.Vision.UseLimelight && Robot.isReal()) {
			LimelightHelpers.setPipelineIndex(Constants.Vision.llAprilTag,
					Constants.Vision.llAprilTagPipelineIndex);
			LimelightHelpers.setPipelineIndex(Constants.Vision.llAprilTagRear,
					Constants.Vision.llAprilTagPipelineIndex);
			LimelightHelpers.setPipelineIndex(Constants.Vision.llPython, Constants.Vision.llPythonPipelineIndex);
		}

		autonCommand.withName("Auto Command").schedule();
		HomeClimber.getHomingCommand().schedule();
	}

	@Override
	public void autonomousPeriodic() {
		autoField.setRobotPose(Drivetrain.getInstance().getPose());
		NetworkTableInstance.getDefault().getTable("PathPlanner").getEntry("Auto Timer").setDouble(timer.get());

		if (timer.get() < pathDelayEntry.getDouble(0)) {
			postAutoStatus("Path Delay");
		}
	}

	@Override
	public void autonomousExit() {
		timer.stop();
	}

	public void postAutoStatus(String status) {
		this.status = status;
	}

	@Override
	public void teleopInit() {
		LimelightHelpers.setLEDMode_ForceOn(Constants.Vision.llPython);

		robotContainer.configureDefaultBinds();

		Arm.getInstance().setBrake(true);
		Arm.getInstance().setMotionMagic(Constants.ArmConstants.SetPoints.kIntake);

		Shooter.getInstance().setShooterBrake(true);

		if (Constants.Vision.UseLimelight) {
			LimelightHelpers.setPipelineIndex(Constants.Vision.llAprilTag,
					Constants.Vision.llAprilTagPipelineIndex);
			LimelightHelpers.setPipelineIndex(Constants.Vision.llAprilTagRear,
					Constants.Vision.llAprilTagPipelineIndex);
			LimelightHelpers.setPipelineIndex(Constants.Vision.llPython, Constants.Vision.llPythonPipelineIndex);
		}

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

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {
	}

	@Override
	public void testExit() {
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
			if (falcon.getPosition().getStatus().isError()) {
				DataLogManager.log("TalonFX #" + falcon.getDeviceID() + " has failed to return position");
				new Alert("TalonFX ID #" + falcon.getDeviceID() + " has failed to return position",
						AlertType.ERROR)
						.set(true);
			}
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
		if (falcon.getPosition().getStatus().isError()) {
			DataLogManager.log("TalonFX #" + falcon.getDeviceID() + " has failed to return position");
			new Alert("TalonFX ID #" + falcon.getDeviceID() + " has failed to return position",
					AlertType.ERROR)
					.set(true);
			return true;
		}

		return false;
	}

	/**
	 * Will return false if the CANcoder is verified and connected, true if there is
	 * some error getting position
	 * 
	 * @param CANcoder a CANcoder
	 * @return false if the position is available, true if not available
	 */
	public static boolean verifyCANcoders(CANcoder CANcoder) {
		if (CANcoder.getPosition().getStatus().isError()) {
			DataLogManager.log("CANcoder #" + CANcoder.getDeviceID() + " has failed to return position");
			new Alert("CANcoder ID #" + CANcoder.getDeviceID() + " has failed to return position",
					AlertType.ERROR)
					.set(true);
			return true;
		}

		return false;
	}
}