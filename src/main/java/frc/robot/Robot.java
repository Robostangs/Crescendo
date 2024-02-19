// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.PathPlannerCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain.Drivetrain;

public class Robot extends TimedRobot {
	public SendableChooser<String> mChooser = new SendableChooser<>();
	public static Field2d mField = new Field2d();

	public static boolean atComp = false;
	public static boolean autonomousExited = false;

	public Command autonCommand;

	@Override
	public void robotInit() {
		DataLogManager.start(Constants.logDirectory);

		new RobotContainer();

		SmartDashboard.putData("Field", mField);

		Drivetrain.getInstance().getDaqThread().setThreadPriority(99);

		CommandScheduler.getInstance()
				.onCommandInitialize((action) -> DataLogManager.log(action.getName() + " Command Initialized"));
		CommandScheduler.getInstance()
				.onCommandInterrupt((action) -> DataLogManager.log(action.getName() + " Command Interrupted"));
		CommandScheduler.getInstance()
				.onCommandFinish((action) -> DataLogManager.log(action.getName() + " Command Finished"));

		if (DriverStation.isFMSAttached()) {
			atComp = true;
			DataLogManager.start(Constants.logDirectory);
		}

		DriverStation.silenceJoystickConnectionWarning(true);
	}

	@Override
	public void driverStationConnected() {
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
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
		autonCommand = new PathPlannerCommand("thing", true); // Name doesnt matter

		autonCommand.schedule();
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
		Arm.getInstance().setMotionMagic(Constants.ArmConstants.SetPoints.kSubwoofer);
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
}
