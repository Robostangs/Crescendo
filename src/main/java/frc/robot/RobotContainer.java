// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.intake.IntakeManual;
import frc.robot.commands.intake.NoteAlign;
import frc.robot.commands.shooter.TargetShoot;

public class RobotContainer {

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController xDrive = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain mDrivetrain = CommandSwerveDrivetrain.getInstance(); // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(DrivetrainConstants.MAX_SPEED * 0.08).withRotationalDeadband(DrivetrainConstants.MAX_ANGULAR_RATE * 0.08) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(DrivetrainConstants.MAX_SPEED);

  private void configureBindings() {
    /* DRIVETRAIN */
    mDrivetrain.setDefaultCommand(
        mDrivetrain.applyRequest(() -> drive.withVelocityX(-xDrive.getLeftY() * DrivetrainConstants.MAX_SPEED)
            .withVelocityY(-xDrive.getLeftX() * DrivetrainConstants.MAX_SPEED) // Drive left with negative X (left)
            .withRotationalRate(-xDrive.getRightX() * DrivetrainConstants.MAX_ANGULAR_RATE) // Drive counterclockwise with negative X (left)
        ));

    if (Utils.isSimulation()) {
      mDrivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    mDrivetrain.registerTelemetry(logger::telemeterize);

    /* KEYBINDS START */

    xDrive.pov(0).onTrue(mDrivetrain.runOnce(() -> mDrivetrain.seedFieldRelative())); // UP - ZERO GYRO
    xDrive.pov(90).whileTrue(mDrivetrain.applyRequest(() -> brake)); // RIGHT - LOCK WHEELS
    xDrive.pov(180).whileTrue(mDrivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-xDrive.getLeftY(), -xDrive.getLeftX())))); // DOWN - POINT WHEELS

    xDrive.axisGreaterThan(2, 0.9).whileTrue(new IntakeManual()); // RT - INTAKE
    xDrive.leftBumper().whileTrue(new NoteAlign(() -> xDrive.getLeftX()).alongWith(new IntakeManual())); // LB - ALIGN TO NOTE & INTAKE

    xDrive.rightBumper().whileTrue(new TargetShoot(() -> xDrive.getLeftX(), () -> xDrive.getLeftY())); // RB - TARGET SHOOT

    /* KEYBINDS END */
  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
