package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Commands.Shooter.AimAndShoot;
import frc.robot.Commands.Shooter.FeedAndShoot;
import frc.robot.Commands.Shooter.FineAdjust;
import frc.robot.Commands.Shooter.SetPoint;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Drivetrain;

public class RobotContainer {
  private final CommandXboxController xDrive = new CommandXboxController(0);
  private final CommandXboxController Manip = new CommandXboxController(1);
  private final GenericHID simController = new GenericHID(3);

  private final Drivetrain drivetrain = Drivetrain.getInstance();
  private final Arm mArm = Arm.getInstance();

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(Constants.OperatorConstants.deadband)
      .withRotationalDeadband(Constants.OperatorConstants.rotationalDeadband)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric();

  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger;
  public Field2d field;

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive
            .withVelocityX(-xDrive.getLeftY() * Constants.SwerveConstants.kMaxSpeedMetersPerSecond)
            .withVelocityY(-xDrive.getLeftX() * Constants.SwerveConstants.kMaxSpeedMetersPerSecond)
            .withRotationalRate(
                -xDrive.getRightX() * Constants.SwerveConstants.kMaxAngularSpeedMetersPerSecond)));
                /* TODO: fix again */
            // .withSlowDown(xDrive.getHID().getRightBumper(), 0.5)).ignoringDisable(true));

    xDrive.a().whileTrue(drivetrain.applyRequest(() -> brake));
    xDrive.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-xDrive.getLeftY(), -xDrive.getLeftX()))));

    xDrive.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    xDrive.x().whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(1).withVelocityY(1)));

    // Manip.y().whileTrue(new SetPoint(Constants.ArmConstants.SetPoints.kAmp));
    // Manip.x().whileTrue(new
    // SetPoint(Constants.ArmConstants.SetPoints.kSpeakerClosestPoint));

    // Manip.y().whileTrue(new RunCommand(() ->
    // mShooter.SetRpm(SmartDashboard.getNumber("Shooter/left motor/setRpm", 0),
    // SmartDashboard.getNumber("Shooter/right motor/setRpm", 0)),
    // mShooter).andThen(() -> mShooter.stop(), mShooter));

    Manip.b().whileTrue(new SetPoint(0));
    Manip.a().whileTrue(new SetPoint(Constants.ArmConstants.SetPoints.kSpeaker));

    Manip.leftBumper().whileTrue(new FeedAndShoot());

    // mArm.setDefaultCommand(new AimAndShoot());

    // Manip.leftBumper().whileTrue(new RunCommand(() -> mShooter.loadPiece()));

    mArm.setDefaultCommand(new FineAdjust(() -> -Manip.getRightY()));

    Manip.x().whileTrue(new AimAndShoot());

    // mShooter.setDefaultCommand(new RunCommand(() -> System.out.println("taking
    // shooter thing"), mShooter));

    /* Set Sim Binds */
    if (Robot.isSimulation()) {
      drivetrain.setDefaultCommand(drivetrain
          .applyRequest(() -> drive
              .withVelocityX(
                  -simController.getRawAxis(1) * Constants.SwerveConstants.kMaxSpeedMetersPerSecond)
              .withVelocityY(
                  -simController.getRawAxis(0) * Constants.SwerveConstants.kMaxSpeedMetersPerSecond)
              .withRotationalRate(-simController.getRawAxis(3)
                  * Constants.SwerveConstants.kMaxAngularSpeedMetersPerSecond))
            //   .withSlowDown(simController.getRawButton(5), Constants2.SwerveConstants.slowDownMultiplier))
          .withName("xDrive"));

      // new Trigger(() ->
      // simController.getRawButtonPressed(1)).onTrue(drivetrain.applyRequest(() ->
      // brake));

      new Trigger(() -> simController.getRawButtonPressed(1))
          .whileTrue(new SetPoint(Constants.ArmConstants.SetPoints.kSpeakerClosestPoint));

      new Trigger(() -> simController.getRawButtonPressed(2))
          .whileTrue(new SetPoint(Constants.ArmConstants.SetPoints.kAmp));

      new Trigger(() -> simController.getRawButtonPressed(3))
          .whileTrue(new SetPoint(Constants.ArmConstants.SetPoints.kSpeaker));

      new Trigger(() -> simController.getRawButtonPressed(4)).whileTrue(new AimAndShoot());

      // new Trigger(() -> simController.getRawButtonPressed(2))
      // .onTrue(new InstantCommand(() -> drivetrain.seedFieldRelative()));

      // new Trigger(() -> simController.getRawButtonPressed(3))
      // .onTrue(new
      // SetPoint(Constants.ArmConstants.SetPoints.kSpeakerClosestPoint).withTimeout(1));

      // new Trigger(() -> simController.getRawButtonPressed(3)).onTrue(new
      // SetPoint(0).withTimeout(1));

      // new Trigger(() -> simController.getRawButtonPressed(3))
      // .whileTrue(drivetrain.applyRequest(() ->
      // forwardStraight.withVelocityX(1).withVelocityY(0)));

      mArm.setDefaultCommand(new FineAdjust(() -> -simController.getRawAxis(2)));

      // new Trigger(() -> simController.getRawButtonPressed(4)).onTrue(new
      // InstantCommand(Intake.toggleDeploy));

    }
  }

  public RobotContainer() {
    logger = new Telemetry();
    field = Robot.mField;
    drivetrain.registerTelemetry((telemetry) -> logger.telemeterize(telemetry));
    configureBindings();
  }
}