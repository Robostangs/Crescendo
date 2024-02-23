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
import frc.robot.commands.AutoCommands.PathPlannerCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivetrain.Drivetrain;

public class Robot extends TimedRobot {
	public SendableChooser<String> mChooser = new SendableChooser<>();
	public static Field2d mField = new Field2d();
	public static PowerDistribution pdh = new PowerDistribution();

	public static RobotContainer robotContainer;
	public static boolean atComp = false;
	public static boolean autonomousExited = false;

	public Command autonCommand;

	@Override
	public void robotInit() {
		robotContainer = new RobotContainer();
		// DataLogManager.start(Constants.logDirectory);

		SmartDashboard.putData("Field", mField);

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
		Intake.getInstance().removeDefaultCommand();
		Shooter.getInstance().removeDefaultCommand();
		Arm.getInstance().removeDefaultCommand();
		Drivetrain.getInstance().removeDefaultCommand();

		autonCommand = new PathPlannerCommand("thing", false); // Name doesnt matter

		autonCommand.schedule();
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void autonomousExit() {
		robotContainer.configureDefaultBinds();
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