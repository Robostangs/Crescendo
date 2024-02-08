// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.PathPlannerCommand;
import frc.robot.Subsystems.Shooter;
import frc.robot.Vision.LimelightHelpers;

public class Robot extends TimedRobot {
  public SendableChooser<String> mChooser = new SendableChooser<>();
  public static Field2d mField = new Field2d();

  public static boolean atComp = false;
  public static boolean autonomousExited = false;

  @Override
  public void robotInit() {
    DataLogManager.start();

    new RobotContainer();

    SmartDashboard.putData("Field", mField);

    mChooser.setDefaultOption("Do Nothing", "null");
    AutoBuilder.getAllAutoNames().forEach((name) -> mChooser.addOption(name, name));
    SmartDashboard.putData("Auton Chooser", mChooser);

    // Swerve.getInstance().getDaqThread().setThreadPriority(99);

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

  @SuppressWarnings("unused")

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    /* TODO: make sure this works */
    // if (Constants2.Vision.UseLimelight && Robot.isReal()) {

    // var lastResult =
    // LimelightHelpers.getLatestResults(Constants.Vision.llAprilTag).targetingResults;
    // Pose2d llPose = lastResult.getBotPose2d_wpiBlue();

    // if (LimelightHelpers.getTid("limelight") != -1) {
    // Drivetrain.getInstance().addVisionMeasurement(llPose,
    // Timer.getFPGATimestamp());
    // }

    // if (!autonomousExited) {
    // PathPlannerCommand.publishTrajectory(mChooser.getSelected());
    // }
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
    new PathPlannerCommand(mChooser.getSelected(), true).schedule();
  }

  @Override
  public void autonomousPeriodic() {
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