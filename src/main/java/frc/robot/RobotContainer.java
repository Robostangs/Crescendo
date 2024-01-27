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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.commands.NoteAlign;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // 6 meters per second desired top speed
  private double MaxAngularRate = 8 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController xDrive = new CommandXboxController(0); // My joystick
  private final CommandXboxController xManip = new CommandXboxController(1); // My joystick
  private final CommandSwerveDrivetrain mDrivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.08).withRotationalDeadband(MaxAngularRate * 0.08) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    /* DRIVETRAIN */
    mDrivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        mDrivetrain.applyRequest(() -> drive.withVelocityX(-xDrive.getLeftY() * MaxSpeed) // Drive forward with
                                                                                          // negative Y (forward)
            .withVelocityY(-xDrive.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-xDrive.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    xDrive.a().whileTrue(mDrivetrain.applyRequest(() -> brake));
    xDrive.b().whileTrue(mDrivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-xDrive.getLeftY(), -xDrive.getLeftX()))));

    // reset the field-centric heading on left bumper press
    xDrive.leftBumper().onTrue(mDrivetrain.runOnce(() -> mDrivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      mDrivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    mDrivetrain.registerTelemetry(logger::telemeterize);

    /* NOTE FINDER */
    xDrive.x().whileTrue(new NoteAlign(() -> xDrive.getLeftX(), () -> xDrive.getLeftY(), MaxSpeed));
  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
