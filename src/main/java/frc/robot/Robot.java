// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.PathPlannerCommand;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Drivetrain.Drivetrain;
import frc.robot.Vision.LimelightHelpers;

public class Robot extends TimedRobot {
	public SendableChooser<String> mChooser = new SendableChooser<>();
	public static Field2d mField = new Field2d();
	private Timer timer;
	private Command auton;

	public static boolean atComp = false;
	public static boolean autonomousExited = false;

	@Override
	public void robotInit() {
		DataLogManager.start(Constants.logDirectory);

		new RobotContainer();

		SmartDashboard.putData("Field", mField);

		mChooser.setDefaultOption("Do Nothing", "null");
		AutoBuilder.getAllAutoNames().forEach((name) -> mChooser.addOption(name, name));
		SmartDashboard.putData("Auton Chooser", mChooser);

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

		if (DriverStation.isFMSAttached()) {
			atComp = true;
			DataLogManager.start(Constants.logDirectory);
		}

		DriverStation.silenceJoystickConnectionWarning(true);
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();

		PathPlannerCommand.publishTrajectory(mChooser.getSelected());
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
		auton = new PathPlannerCommand(mChooser.getSelected(), true);
		auton.schedule();
		timer.restart();
	}

	@Override
	public void autonomousPeriodic() {
		NetworkTableInstance.getDefault().getTable("PathPlanner").getEntry("Auto Timer").setDouble(timer.get());
		if (!CommandScheduler.getInstance().isScheduled(auton)) {
			timer.stop();
		}
	}

	@Override
	public void autonomousExit() {
		if (atComp) {
			PathPlannerCommand.unpublishTrajectory();
			autonomousExited = true;
		}
	}

	@Override
	public void teleopInit() {
		Shooter.getInstance().setBrakeMode(false);
	}

	@Override
	public void teleopPeriodic() {
	}

	@Override
	public void teleopExit() {
		Shooter.getInstance().setBrakeMode(true);
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
}