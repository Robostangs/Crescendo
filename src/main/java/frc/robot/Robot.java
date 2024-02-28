// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Vision.LimelightHelpers;
import frc.robot.commands.AutoCommands.PathPlannerCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Drivetrain.Drivetrain;

public class Robot extends TimedRobot {
	public SendableChooser<String> startingPose = new SendableChooser<>();
	public SendableChooser<String> autoChooser = new SendableChooser<>();
	public SendableChooser<Boolean> autoShoot = new SendableChooser<>();
	public static SendableChooser<Boolean> intakeAlwaysOut = new SendableChooser<>();

	public static ShuffleboardTab autoTab;
	public static ShuffleboardTab teleopTab;

	public static Field2d mField = new Field2d();
	public static PowerDistribution pdh = new PowerDistribution();
	public static Timer timer = new Timer();

	public static RobotContainer robotContainer;
	public static boolean atComp = false;
	public static boolean autonomousExited = false;

	public PathPlannerCommand autonCommand;

	@SuppressWarnings("resource")
	@Override
	public void robotInit() {
		autoTab = Shuffleboard.getTab("Auto");
		teleopTab = Shuffleboard.getTab("Teleop");
		
		robotContainer = new RobotContainer();

		SmartDashboard.putData("Field", mField);

		Drivetrain.getInstance().getDaqThread().setThreadPriority(99);

		startingPose.setDefaultOption("Shoot Only", "null");
		startingPose.addOption("Left", "left");
		startingPose.addOption("Center", "center");
		startingPose.addOption("Right", "right");
		startingPose.addOption("All Close Notes", "all close pieces");
		startingPose.addOption("Center Line Sprint", "centerline sprint");
		startingPose.addOption("Center 4 Piece", "center 4 piece");
		
		autoChooser.setDefaultOption("None Selected", "");
		autoChooser.addOption("1 Piece", " 1 piece");
		autoChooser.addOption("2 Piece", " 2 piece");
		autoChooser.addOption("3 Piece", " 3 piece");
		
		autoShoot.setDefaultOption("Shoot At Start", true);
		autoShoot.addOption("Dont Shoot At Start", false);

		intakeAlwaysOut.setDefaultOption("Intake Always Out", true);
		intakeAlwaysOut.addOption("Intake Retracts When Holding", false);
		
		// SmartDashboard.putData("Auto/Starting Pose Selector", startingPose);
		// SmartDashboard.putData("Auto/Trajectory Chooser", autoChooser);
		// SmartDashboard.putData("Auto/Shoot Chooser", autoShoot);

		autoTab.add("Starting Pose Selector", startingPose).withSize(2, 1).withPosition(0, 0).withWidget(BuiltInWidgets.kComboBoxChooser);
		autoTab.add("Path Selector", autoChooser).withSize(2, 1).withPosition(0, 1)
				.withWidget(BuiltInWidgets.kComboBoxChooser);
		autoTab.add("Shoot Selector", autoShoot).withSize(2, 1).withPosition(0, 2)
				.withWidget(BuiltInWidgets.kComboBoxChooser);
		autoTab.add("Field", mField).withSize(6, 4).withPosition(4, 0).withWidget(BuiltInWidgets.kField);
		autoTab.addNumber("Inacuracy",
				() -> NetworkTableInstance.getDefault().getTable("PathPlanner").getEntry("inaccuracy").getDouble(0.0))
				.withSize(1, 1).withPosition(2, 0);
		autoTab.addNumber("Auto Timer",
				() -> timer.get())
				.withSize(1, 1).withPosition(3, 0);

		autoTab.add("Intake Always Out", intakeAlwaysOut).withSize(2, 1).withPosition(0, 3)
				.withWidget(BuiltInWidgets.kComboBoxChooser);

		Shuffleboard.selectTab(autoTab.getTitle());

		// teleopTab.getLayout("", )


		if (DriverStation.isFMSAttached()) {
			atComp = true;
			DataLogManager.start(Constants.logDirectory);
			CommandScheduler.getInstance()
					.onCommandInitialize((action) -> DataLogManager.log(action.getName() + "Command Initialized"));
			CommandScheduler.getInstance()
					.onCommandInterrupt((action) -> DataLogManager.log(action.getName() + "Command Interrupted"));
			CommandScheduler.getInstance()
					.onCommandFinish((action) -> DataLogManager.log(action.getName() + "Command Finished"));

		} else {
			CommandScheduler.getInstance()
					.onCommandInitialize((action) -> System.out.println(action.getName() + " Command Initialized"));
			CommandScheduler.getInstance()
					.onCommandInterrupt((action) -> System.out.println(action.getName() + " Command Interrupted"));
			CommandScheduler.getInstance()
					.onCommandFinish((action) -> System.out.println(action.getName() + " Command Finished"));
		}

		DriverStation.silenceJoystickConnectionWarning(true);
	}

	@Override
	public void driverStationConnected() {
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		// LoggyThingManager.getInstance().periodic();


 		if (Intake.getInstance().getShooterSensor() && DriverStation.isEnabled()) {

			// LEDs will blink when the arm is at the right setpoint to score and swerve is facing correct angle
			if (Arm.getInstance().isInRangeOfTarget(Arm.getInstance().calculateArmSetpoint())
					&& Drivetrain.getInstance().isInRangeOfTarget(30)) {
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
	}

	@Override
	public void disabledExit() {
	}

	@Override
	public void autonomousInit() {
		Shuffleboard.selectTab(autoTab.getTitle());

		robotContainer.removeDefaultCommands();

		// this will end up being something like "left 1 piece" or "right 3 piece"
		// System.out.println("Path to run: " +  startingPose.getSelected() + " " + autoChooser.getSelected());

		if (autonCommand == null) {
			autonCommand = new PathPlannerCommand(startingPose.getSelected() + autoChooser.getSelected(), autoShoot.getSelected());
		}

		timer.restart();

		autonCommand.schedule();
	}

	@Override
	public void autonomousPeriodic() {
		if (autonCommand.isFinished()) {
			timer.stop();
		}

		NetworkTableInstance.getDefault().getTable("PathPlanner").getEntry("Auto Timer").setDouble(timer.get());
	}

	@Override
	public void autonomousExit() {
		robotContainer.configureDefaultBinds();
		timer.stop();
	}

	@Override
	public void teleopInit() {
		Arm.getInstance().setBrake(false);
		Arm.getInstance().setMotionMagic(Constants.ArmConstants.SetPoints.kIntake);

		if (Constants.Vision.UseLimelight) {
			LimelightHelpers.setPipelineIndex(Constants.Vision.llAprilTag,
					Constants.Vision.llAprilTagPipelineIndex);
			LimelightHelpers.setPipelineIndex(Constants.Vision.llAprilTagRear,
					Constants.Vision.llAprilTagPipelineIndex);
			LimelightHelpers.setPipelineIndex(Constants.Vision.llPython, Constants.Vision.llPythonPipelineIndex);
		}

		PathPlannerCommand.publishTrajectory(null);
		// PathPlannerCommand.unpublishTrajectory();

		// teleopTab.add("Field", mField).withSize(5, 3).withPosition(4, 0);
		Shuffleboard.selectTab(teleopTab.getTitle());
		// Shuffleboard.selectTab(0);
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