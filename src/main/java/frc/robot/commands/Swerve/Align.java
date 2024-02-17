package frc.robot.commands.Swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Vision.LimelightHelpers;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.subsystems.Drivetrain.SwerveRequest;

public class Align extends Command {
    private Drivetrain mDrivetrain = Drivetrain.getInstance();
    private Intake mIntake = Intake.getInstance();

    private SwerveRequest.FieldCentricFacingAngle drive;
    private SwerveRequest.FieldCentric drive2;
    private Timer timer;

    private Supplier<Double> translateX, translateY, howManyBabiesOnBoard;
    private Supplier<Rotation2d> getTargetRotation;
    private boolean note;

    /** Test this soon af */
    public Align(Supplier<Double> translateX, Supplier<Double> translateY, Supplier<Double> howManyBabiesOnBoard,
            boolean note) {
        this.addRequirements(mDrivetrain);
        this.setName("Align");

        if (howManyBabiesOnBoard == null) {
            this.howManyBabiesOnBoard = () -> 0.0;
        }

        timer = new Timer();
        this.note = note;

        this.translateX = translateX;
        this.translateY = translateY;
        this.howManyBabiesOnBoard = howManyBabiesOnBoard;

        if (note) {
            getTargetRotation = () -> {
                return mDrivetrain.getPose().getRotation()
                        .plus(Rotation2d.fromDegrees(LimelightHelpers.getTX(Constants.Vision.llAprilTagRear)));
            };
            // if (LimelightHelpers.getTX(Constants.Vision.llAprilTagRear) != 0) {
            // return mDrivetrain.getPose().getRotation()
            // .plus(Rotation2d.fromDegrees(LimelightHelpers.getTX(Constants.Vision.llAprilTagRear)));
            // } else {
            // return Rotation2d
            // .fromRadians(-Math.atan2(Constants.Vision.SpeakerCoords[1] -
            // mDrivetrain.getPose().getY(),
            // Constants.Vision.SpeakerCoords[0] - mDrivetrain.getPose().getX()));
            // }
            // };
            this.addRequirements(mIntake);
        } else {
            getTargetRotation = () -> {
                // return mDrivetrain.getPose().getRotation()
                // .minus(Rotation2d.fromDegrees(LimelightHelpers.getTX(Constants.Vision.llAprilTagRear)));
                // };
                // if (LimelightHelpers.getTid(Constants.Vision.llAprilTagRear) != -1) {
                // return mDrivetrain.getPose().getRotation()
                // .minus(Rotation2d.fromDegrees(LimelightHelpers.getTX(Constants.Vision.llAprilTagRear)));
                // } else {
                // return Rotation2d
                // .fromRadians(-Math.atan2(mDrivetrain.getPose().getY() -
                // Constants.Vision.SpeakerCoords[1],
                // Constants.Vision.SpeakerCoords[0] - mDrivetrain.getPose().getX()));
                // }

                if (LimelightHelpers.getTid(Constants.Vision.llAprilTagRear) == -1) {
                    return Rotation2d.fromDegrees(0);
                } else {
                    return mDrivetrain.getPose().getRotation()
                            .minus(Rotation2d.fromDegrees(LimelightHelpers.getTX(Constants.Vision.llAprilTagRear)));
                }
            };
        }
    }

    @Override
    public void initialize() {
        drive = new SwerveRequest.FieldCentricFacingAngle();
        drive.Deadband = Constants.OperatorConstants.deadband;
        drive.RotationalDeadband = Constants.OperatorConstants.rotationalDeadband * 2;

        drive2 = new SwerveRequest.FieldCentric();
        drive2.Deadband = Constants.OperatorConstants.deadband;
        drive2.RotationalDeadband = Constants.OperatorConstants.rotationalDeadband * 2;
        timer.restart();

        if (note) {
            LimelightHelpers.setPipelineIndex(Constants.Vision.llAprilTagRear, 1);
        } else {
            LimelightHelpers.setPipelineIndex(Constants.Vision.llAprilTagRear, 2);
        }
    }

    @Override
    public void execute() {
        drive.RotationalDeadband = 0;

        if (note) {
            mIntake.setExtend(true);
            if (timer.get() > Constants.IntakeConstants.kDeployTimeSeconds) {
                mIntake.setIntake(1);
            }
        }

        double rotationError = drive.TargetDirection.getDegrees() - getTargetRotation.get().getDegrees();

        SmartDashboard.putNumber("Swerve/Rotation Error", rotationError);

        drive
                .withVelocityX(-translateY.get()
                        * Constants.SwerveConstants.kMaxSpeedMetersPerSecond)
                .withVelocityY(-translateX.get()
                        * Constants.SwerveConstants.kMaxSpeedMetersPerSecond)
                .withSlowDown(1 - howManyBabiesOnBoard.get());

        // drive2
        // .withVelocityX(-translateY.get()
        // * Constants.SwerveConstants.kMaxSpeedMetersPerSecond)
        // .withVelocityY(-translateX.get()
        // * Constants.SwerveConstants.kMaxSpeedMetersPerSecond)
        // .withSlowDown(1 - howManyBabiesOnBoard.get());

        if (Math.abs(rotationError) < 5) {
            SmartDashboard.putString("Swerve/Rotation Status", "In Position");
            // mDrivetrain.setControl(drive2);
        } else {
            SmartDashboard.putString("Swerve/Rotation Status", "Travelling");
            drive.TargetDirection = getTargetRotation.get();
        }

        mDrivetrain.setControl(drive);
        SmartDashboard.putNumber("Swerve/Rotation Target", drive.TargetDirection.getDegrees());
    }

    @Override
    public void end(boolean interrupted) {
        // mDrivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
        if (note) {
            mIntake.setIntake(0);
            mIntake.setExtend(false);
        }
    }

    // @Override
    // public void execute() {
    // drive.RotationalDeadband = 0;

    // if (note) {
    // mIntake.setExtend(true);
    // if (timer.get() > Constants.IntakeConstants.kDeployTimeSeconds) {
    // mIntake.setIntake(1);
    // }
    // }

    // SmartDashboard.putNumber("Swerve/Rotation Error",
    // drive.TargetDirection.getDegrees() - getTargetRotation.get().getDegrees());

    // drive
    // .withVelocityX(-translateY.get()
    // * Constants.SwerveConstants.kMaxSpeedMetersPerSecond)
    // .withVelocityY(-translateX.get()
    // * Constants.SwerveConstants.kMaxSpeedMetersPerSecond)
    // .withSlowDown(1 - howManyBabiesOnBoard.get());

    // drive2
    // .withVelocityX(-translateY.get()
    // * Constants.SwerveConstants.kMaxSpeedMetersPerSecond)
    // .withVelocityY(-translateX.get()
    // * Constants.SwerveConstants.kMaxSpeedMetersPerSecond)
    // .withSlowDown(1 - howManyBabiesOnBoard.get());

    // if (Math.abs(drive.TargetDirection.getDegrees() -
    // getTargetRotation.get().getDegrees()) < 5) {
    // SmartDashboard.putString("Swerve/Rotation Status", "In Position");
    // mDrivetrain.setControl(drive2);
    // } else {
    // SmartDashboard.putString("Swerve/Rotation Status", "Travelling");
    // drive.TargetDirection = getTargetRotation.get();
    // mDrivetrain.setControl(drive);
    // }

    // SmartDashboard.putNumber("Swerve/Rotation Target",
    // drive.TargetDirection.getDegrees());
    // }
}
