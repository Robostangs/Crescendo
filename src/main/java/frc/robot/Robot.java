// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

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
import frc.robot.Vision.LimelightHelpers;
import frc.robot.commands.AutoCommands.AutoManager;
import frc.robot.commands.AutoCommands.PathPlannerCommand;
import frc.robot.commands.Swerve.Align;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.subsystems.Drivetrain.SwerveRequest;

public class Robot extends TimedRobot {
	public static SendableChooser<String> startingPose = new SendableChooser<>();
	public static SendableChooser<String> autoChooser = new SendableChooser<>();
	public static SendableChooser<Boolean> autoShoot = new SendableChooser<>();
	public static SendableChooser<Boolean> intakeAlwaysOut = new SendableChooser<>();
	public static NetworkTableEntry pathDelayEntry;

	public static ShuffleboardTab autoTab, teleopTab;

	public static Field2d teleopField = new Field2d(), autoField = new Field2d();
	public static PowerDistribution pdh = new PowerDistribution();
	public static Timer timer = new Timer();
	public static RobotContainer robotContainer;

	public static boolean atComp = false;

	public SequentialCommandGroup autonCommand;
	public Command pathPlannerCommand;
	public static AutoManager autoManager;

	@Override
	public void robotInit() {
		autoTab = Shuffleboard.getTab("Auto");
		teleopTab = Shuffleboard.getTab("Teleop");

		robotContainer = new RobotContainer();

		Drivetrain.getInstance().getDaqThread().setThreadPriority(99);

		if (DriverStation.isFMSAttached()) {
			atComp = true;
			DataLogManager.start(Constants.logDirectory);
			CommandScheduler.getInstance()
					.onCommandInitialize((action) -> DataLogManager.log(action.getName() + "Command Initialized"));
			CommandScheduler.getInstance()
					.onCommandInterrupt((action) -> DataLogManager.log(action.getName() + "Command Interrupted"));
			CommandScheduler.getInstance()
					.onCommandFinish((action) -> DataLogManager.log(action.getName() + "Command Finished"));

			Shuffleboard.startRecording();
		} else {
			CommandScheduler.getInstance()
					.onCommandInitialize((action) -> System.out.println(action.getName() + " Command Initialized"));
			CommandScheduler.getInstance()
					.onCommandInterrupt((action) -> System.out.println(action.getName() + " Command Interrupted"));
			CommandScheduler.getInstance()
					.onCommandFinish((action) -> System.out.println(action.getName() + " Command Finished"));
		}

		SmartDashboard.putData("Field", teleopField);

		startingPose.setDefaultOption("Center", "center"); // center
		startingPose.addOption("Amp Side", "amp"); // left
		startingPose.addOption("Stage Side", "stage"); // right

		// this could be literally whatever it doesnt matter cuz the path is null and
		// means nothing
		autoChooser.setDefaultOption("Sit and Shit", " null");
		autoChooser.addOption("Simple Reverse", "back-up");
		autoChooser.addOption("1 Piece", " 1 piece");
		autoChooser.addOption("2 Piece", " 2 piece");
		autoChooser.addOption("3 Piece", " 3 piece");
		autoChooser.addOption("4 Piece (Center Only)", " 4 piece");
		autoChooser.addOption("Center Line Sprint (Center Only)", " centerline sprint");
		autoChooser.addOption("All Close Notes (Center Only)", " all close notes");
		autoChooser.addOption("All Close Notes Fast", " all close notes shoot in place");
		autoChooser.addOption("Close 2 Piece (No Center)", " close 2 piece");
		autoChooser.addOption("Far 1 Piece (No Center)", " far 1 piece");
		autoChooser.addOption("Far 1 Piece (No Center)", " far 1 piece");
		autoChooser.addOption("Far 2 Piece (No Center)", " far 2 piece");

		autoShoot.setDefaultOption("Shoot At Start", true);
		autoShoot.addOption("Dont Shoot At Start", false);

		intakeAlwaysOut.setDefaultOption("Intake Always Out", true);
		intakeAlwaysOut.addOption("Intake Retracts When Holding", false);

		autoTab.add("Starting Pose Selector", startingPose).withSize(2, 1).withPosition(0, 0)
				.withWidget(BuiltInWidgets.kComboBoxChooser);
		autoTab.add("Path Selector", autoChooser).withSize(2, 1).withPosition(0, 1)
				.withWidget(BuiltInWidgets.kComboBoxChooser);
		autoTab.add("Shoot Selector", autoShoot).withSize(2, 1).withPosition(0, 2)
				.withWidget(BuiltInWidgets.kComboBoxChooser);
		autoTab.add("Intake Always Out", intakeAlwaysOut).withSize(2, 1).withPosition(0, 3)
				.withWidget(BuiltInWidgets.kComboBoxChooser);
		autoTab.addNumber("Auto Countdown", () -> Timer.getMatchTime()).withSize(2, 2).withPosition(2, 2)
				.withWidget(BuiltInWidgets.kDial)
				.withProperties(Map.of("min", -1, "max", 15, "show value", true));
		autoTab.add("Path Delay", 0).withSize(2, 1).withWidget(BuiltInWidgets.kNumberSlider).withPosition(0, 4)
				.withProperties(Map.of("min", 0, "max", 15, "block increment", 1));

		pathDelayEntry = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Auto")
				.getEntry("Path Delay");

		teleopTab.addNumber("Match Time", () -> Timer.getMatchTime()).withSize(2, 2).withPosition(2, 0)
				.withWidget(BuiltInWidgets.kDial)
				.withProperties(Map.of("min", -1, "max", 135, "show value", true));

		teleopTab.addBoolean("Holding", () -> Intake.getInstance().getHolding()).withSize(2, 2).withPosition(0, 2)
				.withWidget(BuiltInWidgets.kBooleanBox);
		teleopTab.addBoolean("Ready To Shoot", () -> Shooter.getInstance().readyToShootAdvanced()).withSize(2, 2)
				.withPosition(0, 0);

		teleopTab.addString("Selected Climber", () -> Climber.getInstance().getLeftSelected() ? "Left" : "Right")
				.withSize(2, 2)
				.withPosition(2, 2).withWidget(BuiltInWidgets.kTextView);

		if (Robot.isReal() && Constants.Vision.UseLimelight) {
			try {
				// front camera (intake cam)
				teleopTab.add(new HttpCamera(Constants.Vision.llPython, Constants.Vision.llPythonIP))
						.withWidget(BuiltInWidgets.kCameraStream).withSize(5, 4).withPosition(4, 0)
						.withProperties(Map.of("Show Crosshair", false, "Show Controls", false));
				// rear camera (shooting cam)
				teleopTab.add(new HttpCamera(Constants.Vision.llAprilTagRear, Constants.Vision.llAprilTagRearIP))
						.withWidget(BuiltInWidgets.kCameraStream).withSize(4, 4).withPosition(9, 0)
						.withProperties(Map.of("Show Crosshair", true, "Show Controls", false));

				// front camera (intake cam)
				autoTab.add(new HttpCamera(Constants.Vision.llPython, Constants.Vision.llPythonIP))
						.withWidget(BuiltInWidgets.kCameraStream).withSize(5, 4).withPosition(4, 0)
						.withProperties(Map.of("Show Crosshair", false, "Show Controls", false));
				// rear camera (shooting cam)
				autoTab.add(new HttpCamera(Constants.Vision.llAprilTagRear, Constants.Vision.llAprilTagRearIP))
						.withWidget(BuiltInWidgets.kCameraStream).withSize(4, 4).withPosition(9, 0)
						.withProperties(Map.of("Show Crosshair", true, "Show Controls", false));

			} catch (Exception e) {
				System.out.println("Failed to add camera to Shuffleboard");
			}
		}

		Shuffleboard.selectTab(autoTab.getTitle());

		DriverStation.silenceJoystickConnectionWarning(true);

		NamedCommands.registerCommand("align and shoot", new InstantCommand(() -> autoManager.shoot = true)
				.alongWith(new WaitUntilCommand(() -> autoManager.shoot == false).raceWith(new Align(false))));

		// use this for on the fly shooting
		NamedCommands.registerCommand("shoot", new InstantCommand(() -> autoManager.shoot = true)
				.alongWith(new WaitUntilCommand(() -> autoManager.shoot == false)));
	}

	@Override
	public void driverStationConnected() {
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();

		if (Intake.getInstance().getShooterSensor() && DriverStation.isEnabled()) {

			// LEDs will blink when the arm is at the right setpoint to score and swerve is
			// facing correct angle
			if (Arm.getInstance().isInRangeOfTarget(Arm.getInstance().calculateArmSetpoint())
					&& Drivetrain.getInstance().isInRangeOfTarget()) {
				LimelightHelpers.setLEDMode_ForceBlink(Constants.Vision.llAprilTag);
				LimelightHelpers.setLEDMode_ForceBlink(Constants.Vision.llAprilTagRear);
			}

			// LEDs will be on when the arm is not at the right setpoint to score, but the
			// shooter is occupied
			else {
				LimelightHelpers.setLEDMode_ForceOn(Constants.Vision.llAprilTag);
				LimelightHelpers.setLEDMode_ForceOn(Constants.Vision.llAprilTagRear);
			}
		}

		// LEDs will be off when the shooter is not occupied
		else {
			LimelightHelpers.setLEDMode_ForceOff(Constants.Vision.llAprilTag);
			LimelightHelpers.setLEDMode_ForceOff(Constants.Vision.llAprilTagRear);
		}
	}

	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
		PathPlannerCommand.publishTrajectory(startingPose.getSelected() + autoChooser.getSelected());
		autoField.setRobotPose(Drivetrain.getInstance().getPose());
	}

	@Override
	public void disabledExit() {
	}

	@Override
	public void autonomousInit() {
		Shuffleboard.selectTab(autoTab.getTitle());
		Shooter.getInstance().setShooterBrake(true);

		if (autoManager == null) {
			autoManager = new AutoManager();
		}

		robotContainer.removeDefaultCommands();

		try {
			pathPlannerCommand = AutoBuilder.buildAuto(startingPose.getSelected() + autoChooser.getSelected());
		} catch (Exception e) {
			if (autoChooser.getSelected().equals("back-up")) {
				pathPlannerCommand = Drivetrain.getInstance()
						.applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(0.5));
			}

			else {
				pathPlannerCommand = new PrintCommand("Null Path");
				System.out.println("Invalid Auto");
			}
		}

		autonCommand = new SequentialCommandGroup(
				new InstantCommand(() -> Robot.autoManager.shoot = autoShoot.getSelected())
						.alongWith(new WaitUntilCommand(() -> Robot.autoManager.shoot == false)),
				new WaitUntilCommand(() -> timer.get() > pathDelayEntry.getDouble(0)),
				pathPlannerCommand);

		autoManager.initialize();
		autonCommand.schedule();
		timer.restart();
	}

	@Override
	public void autonomousPeriodic() {
		autoManager.execute();
		autoField.setRobotPose(Drivetrain.getInstance().getPose());
		NetworkTableInstance.getDefault().getTable("PathPlanner").getEntry("Auto Timer").setDouble(timer.get());

		if (timer.get() < pathDelayEntry.getDouble(0)) {
			autoManager.postAutoStatus("Path Delay");
		}
	}

	@Override
	public void autonomousExit() {
		autoManager.end(false);
		timer.stop();
	}

	@Override
	public void teleopInit() {
		robotContainer.configureDefaultBinds();

		Arm.getInstance().setBrake(false);
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
	}

	@Override
	public void teleopPeriodic() {
	}

	@Override
	public void teleopExit() {
		Arm.getInstance().setBrake(true);
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
}