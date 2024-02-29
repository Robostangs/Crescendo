package frc.robot.commands.Swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Vision.LimelightHelpers;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.subsystems.Drivetrain.SwerveRequest;

public class Align extends Command {
    private Drivetrain mDrivetrain;
    private Intake mIntake;

    private SwerveRequest.FieldCentricFacingAngle drive;
    private Timer timer;

    private Supplier<Double> translateX, translateY, howManyBabiesOnBoard;
    private Supplier<Rotation2d> getTargetRotation;
    private boolean note;

    public Align(boolean note) {
        this(() -> 0.0, () -> 0.0, () -> 0.0, note);
    }


    public Align(Supplier<Double> translateX, Supplier<Double> translateY, Supplier<Double> howManyBabiesOnBoard,
            boolean note) {
        mDrivetrain = Drivetrain.getInstance();
        mIntake = Intake.getInstance();
        this.addRequirements(mDrivetrain);
        if (note) {
            this.setName("Align to Note");
        } else {
            this.setName("Align to Speaker");
        }

        if (howManyBabiesOnBoard == null) {
            this.howManyBabiesOnBoard = () -> 0.0;
        }
        
        else {
            this.howManyBabiesOnBoard = howManyBabiesOnBoard;
        }

        timer = new Timer();
        this.note = note;

        this.translateX = translateX;
        this.translateY = translateY;

        if (note) {
            // if (LimelightHelpers.getTX(Constants.Vision.llPython) == 0) {
            // getTargetRotation = () -> {
            // return mDrivetrain.getPose().getRotation().minus(Rotation2d.fromDegrees(90));
            // };
            // } else {
            getTargetRotation = () -> {
                return mDrivetrain.getPose().getRotation()
                        .minus(Rotation2d.fromDegrees(LimelightHelpers.getTX(Constants.Vision.llPython)));
            };

            this.addRequirements(mIntake);
        } else {
            getTargetRotation = () -> {
                // just use the pose for everything, dont look for the rear april tag

                if (Robot.isRed()) {
                    return Rotation2d
                            .fromRadians(Math.atan2(
                                    mDrivetrain.getPose().getY() - Constants.Vision.SpeakerPoseRed.getY(),
                                    mDrivetrain.getPose().getX() - Constants.Vision.SpeakerPoseRed.getX()));
                }

                else {
                    return Rotation2d
                            .fromRadians(Math.atan2(
                                    mDrivetrain.getPose().getY() - Constants.Vision.SpeakerPoseBlue.getY(),
                                    mDrivetrain.getPose().getX() - Constants.Vision.SpeakerPoseBlue.getX()));
                }
            };
        }
    }

    @Override
    public void initialize() {
        drive = new SwerveRequest.FieldCentricFacingAngle();
        drive.Deadband = Constants.OperatorConstants.deadband;
        drive.RotationalDeadband = Constants.OperatorConstants.rotationalDeadband * 0.1;

        timer.restart();

        if (note) {
            LimelightHelpers.setPipelineIndex(Constants.Vision.llPython, Constants.Vision.llPythonPipelineIndex);
        } else {
            // This pipeline will only look for the Speaker April Tag
            LimelightHelpers.setPipelineIndex(Constants.Vision.llAprilTagRear, 2);
        }
    }

    @Override
    public void execute() {
        if (note) {
            mIntake.setExtend(true);
            if (timer.get() > Constants.IntakeConstants.kDeployTimeSeconds) {
                mIntake.setIntake(1);
                mIntake.setBelt(Constants.IntakeConstants.beltIntakeSpeed);
                SmartDashboard.putString("Intake/Status", "Align and Intaking");
            }
        }

        double rotationError = drive.TargetDirection.getDegrees() - getTargetRotation.get().getDegrees();

        SmartDashboard.putNumber("Swerve/Rotation Error", rotationError);

        if (Math.abs(rotationError) < 5) {
            SmartDashboard.putString("Swerve/Rotation Status", "In Position");
        } else {
            SmartDashboard.putString("Swerve/Rotation Status", "Traveling");
        }

        drive.TargetDirection = getTargetRotation.get();
        SmartDashboard.putNumber("Swerve/Rotation Target", drive.TargetDirection.getDegrees());

        drive
                .withVelocityX(-translateY.get()
                        * Constants.SwerveConstants.kMaxSpeedMetersPerSecond)
                .withVelocityY(-translateX.get()
                        * Constants.SwerveConstants.kMaxSpeedMetersPerSecond)
                .withSlowDown(1 - howManyBabiesOnBoard.get());

        mDrivetrain.setControl(drive);
    }

    @Override
    public void end(boolean interrupted) {
        LimelightHelpers.setPipelineIndex(Constants.Vision.llAprilTagRear, Constants.Vision.llAprilTagPipelineIndex);

        if (note) {
            mIntake.setIntake(0);
            mIntake.setExtend(false);
            mIntake.setHolding(!interrupted);
        }

        mDrivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
    }

    @Override
    public boolean isFinished() {
        if (note) {
            return mIntake.getShooterSensor();
        } else {
            return !mIntake.getHolding();
        }
    }
}
