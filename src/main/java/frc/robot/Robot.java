// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Vision.LimelightHelpers;
import frc.robot.commands.PathPlannerCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain.Drivetrain;

public class Robot extends TimedRobot {

	private Timer timer;
	private Command auton;
	private ShuffleboardTab autonTab;
	private NetworkTable autonTable;
	private NetworkTableEntry closePieceCount, farPieceCount, shoot, startPosition;
	private SendableChooser<String> startPos = new SendableChooser<>();
	private SendableChooser<String> pieceChooser = new SendableChooser<>();


	public static boolean atComp = false;
	public static boolean autonomousExited = false;
	public static boolean isRed = false;
	public static Field2d mField = new Field2d();


	/* TODO: Set this value for every auto */
	public static final boolean shootFirst = true;

	public Command autonCommand;

	@Override
	public void robotInit() {
		if (DriverStation.isFMSAttached()) {
			atComp = true;
			DataLogManager.start(Constants.logDirectory);
		}
		if(atComp){
			Optional<Alliance> ally = DriverStation.getAlliance();
			if (ally.isPresent()) {
				if(ally.get() == Alliance.Blue){

					isRed = true;

				}
				else if(ally.get() == Alliance.Red){
					isRed = true;
			}
			}
		}


		timer = new Timer();
		DataLogManager.start();

		new RobotContainer();

		autonTable = NetworkTableInstance.getDefault().getTable("Autonomous");
		autonTab = Shuffleboard.getTab("Autonomous");

		PathPlannerCommand.registerCommand();

		startPos.setDefaultOption("THE DEFAULT IS it GOES FOREWORD", "go");
		startPos.addOption("Left", "left");
		startPos.addOption("Right", "right");
		startPos.addOption("Center", "center");
		SmartDashboard.putData("Start Position", startPos);
		
		pieceChooser.setDefaultOption("DEFAULT IS ONE", "one");
		pieceChooser.addOption("One", "1");
		pieceChooser.addOption("Two", "2");
		pieceChooser.addOption("Three", "3");
		//TODO MAKE SOMETHING FOR THIS OPTION IN PATHPLANNERCOMMAND
		pieceChooser.addOption("Weird", "uhoh");
		SmartDashboard.putData("Piece Count", pieceChooser);


		// autonTable = NetworkTableInstance.getDefault().getTable("Autonomous");
		// autonTab = Shuffleboard.getTab("Autonomous");
		// SendableChooser<String> startPosition = new SendableChooser<>();

		// autonTab.add("How many close pieces", 0).withWidget(BuiltInWidgets.kNumberSlider)
		// 		.withProperties(Map.of("min", 0, "max", 3, "block increment", 1)).withSize(2, 1);

		// autonTab.add("How many far pieces", 0).withWidget(BuiltInWidgets.kNumberSlider)
		// 		.withProperties(Map.of("min", 0, "max", 3, "block increment", 1)).withSize(2, 1);

		// autonTab.add("Shoot", true).withWidget(BuiltInWidgets.kToggleButton).withSize(1, 1);

		// startPosition.setDefaultOption("CHANGE NOW DEFAULTS TO LEFT", "left");
		// startPosition.addOption("Left", "left");
		// startPosition.addOption("Right", "right");
		// startPosition.addOption("Center", "center");
		// autonTab.add("Start Position", startPosition).withWidget(BuiltInWidgets.kComboBoxChooser).withSize(2, 1);

		this.closePieceCount = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Autonomous")
				.getEntry("How many close pieces");
		this.farPieceCount = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Autonomous")
				.getEntry("How many far pieces");
		this.shoot = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Autonomous")
				.getEntry("Shoot");
		this.startPosition = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Autonomous")
				.getSubTable("Start Position").getEntry("active");
				
		// autonTable = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Autonomous");

		/* All auton information */

	


		// mChooser.setDefaultOption("Do Nothing", "null");
		// AutoBuilder.getAllAutoNames().forEach((name) -> mChooser.addOption(name, name));
		// SmartDashboard.putData("Auton Chooser", mChooser);


		// mChooser.addOption("thing", "thing");

		// autonTable.getEntry("Shoot").setBoolean(true);
		// // NetworkTableInstance.getDefault().getTable("Autonomous").getEntry("Shoot First").setBoolean(true);

		Drivetrain.getInstance().getDaqThread().setThreadPriority(99);

		CommandScheduler.getInstance()
				.onCommandInitialize((action) -> DataLogManager.log(action.getName() + " Command Initialized"));
		CommandScheduler.getInstance()
				.onCommandInterrupt((action) -> DataLogManager.log(action.getName() + " Command Interrupted"));
		CommandScheduler.getInstance()
				.onCommandFinish((action) -> DataLogManager.log(action.getName() + " Command Finished"));

		if (Constants.Vision.UseLimelight) {
			LimelightHelpers.setPipelineIndex(Constants.Vision.llAprilTag, Constants.Vision.llAprilTagPipelineIndex);
			LimelightHelpers.setPipelineIndex(Constants.Vision.llPython, Constants.Vision.llPythonPipelineIndex);
		}

		DriverStation.silenceJoystickConnectionWarning(true);


	}

	@SuppressWarnings("unused")
	@Override
	public void driverStationConnected() {
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();

		// auton = PathPlannerCommand.pickAuto(startPos.getSelected());


		mField.setRobotPose(Drivetrain.getInstance().getPose());

		SmartDashboard.putData("THE Field", mField);
		
		// PathPlannerCommand.publishTrajectory(mChooser.getSelected());
	}

	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
	}

	@Override
	public void disabledExit() {
	}

	@Override
	public void autonomousInit() {
		timer = new Timer(); 
		// auton = new PathPlannerCommand(startPosition.getString("left"), closePieceCount.getInteger(0),
		// 		farPieceCount.getInteger(0), false);

		// auton = PathPlannerCommand.pickAuto(startPos.getSelected());
		auton = PathPlannerCommand.pickAuto(startPos.getSelected(), pieceChooser.getSelected());

		// auton = new PathPlannerCommand(mChooser.getSelected(),
		// this.shoot.getBoolean(false));
		auton.schedule();
		timer.restart();
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void autonomousExit() {
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