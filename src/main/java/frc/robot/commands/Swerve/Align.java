package frc.robot.commands.Swerve;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Vision.LimelightHelpers;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.subsystems.Drivetrain.SwerveRequest;

public class Align extends Command {
    Drivetrain drivetrain;

    SwerveRequest.FieldCentricFacingAngle driveRequest;

    Supplier<Double> translateX, translateY, howManyBabiesOnBoard;
    Supplier<Rotation2d> getTargetRotation;
    boolean note;

    public Align(boolean note) {
        this(() -> 0.0, () -> 0.0, () -> 0.0, note);
    }

    public Align(Supplier<Double> translateX, Supplier<Double> translateY, Supplier<Double> howManyBabiesOnBoard,
            boolean note) {
        drivetrain = Drivetrain.getInstance();

        this.addRequirements(drivetrain);

        if (howManyBabiesOnBoard == null) {
            this.howManyBabiesOnBoard = () -> 0.0;
        }

        else {
            this.howManyBabiesOnBoard = howManyBabiesOnBoard;
        }

        this.note = note;

        this.translateX = translateX;
        this.translateY = translateY;

        if (note) {
            this.setName("Align to Note");

            getTargetRotation = () -> {
                return drivetrain.getPose().getRotation()
                        .minus(Rotation2d.fromDegrees(LimelightHelpers.getTX(Constants.Vision.llPython)));
            };

        } else {
            this.setName("Align to Speaker");

            getTargetRotation = () -> {
                if (Robot.isRed()) {
                    return Rotation2d
                            .fromRadians(Math.atan2(
                                    drivetrain.getPose().getY() - Constants.Vision.SpeakerPoseRed.getY(),
                                    drivetrain.getPose().getX() - Constants.Vision.SpeakerPoseRed.getX()));
                }

                else {
                    return Rotation2d
                            .fromRadians(Math.atan2(
                                    drivetrain.getPose().getY() - Constants.Vision.SpeakerPoseBlue.getY(),
                                    drivetrain.getPose().getX() - Constants.Vision.SpeakerPoseBlue.getX()));
                }
            };
        }
    }

    @Override
    public void initialize() {
        driveRequest = new SwerveRequest.FieldCentricFacingAngle();

        // TODO: cuz faster drivertrain this needs to be redone
        // note align will have smaller error but more important changes in
        // rotation, so if that needs a different pid controller we can make that happen
        // drive.HeadingController = new PhoenixPIDController(4.0, 20, 0.3);
        driveRequest.HeadingController = new PhoenixPIDController(10, 2, 1);

        // this is for tuning and now we can tune the PID controller
        SmartDashboard.putData("Align PID", driveRequest.HeadingController);
        SmartDashboard.putString("Swerve/status", "Aligning");

        driveRequest.Deadband = Constants.OperatorConstants.deadband;
        driveRequest.RotationalDeadband = Constants.OperatorConstants.rotationalDeadband * 0.05;
    }

    @Override
    public void execute() {
        driveRequest.TargetDirection = getTargetRotation.get();
        double rotationError = driveRequest.TargetDirection.getDegrees() - getTargetRotation.get().getDegrees();

        SmartDashboard.putNumber("Swerve/Rotation Error", rotationError);

        driveRequest
                .withVelocityX(-translateY.get()
                        * Constants.SwerveConstants.kMaxSpeedMetersPerSecond)
                .withVelocityY(-translateX.get()
                        * Constants.SwerveConstants.kMaxSpeedMetersPerSecond)
                .withSlowDown(1 - howManyBabiesOnBoard.get());

        drivetrain.setControl(driveRequest);
    }

    @Override
    public void end(boolean interrupted) {
        LimelightHelpers.setPipelineIndex(Constants.Vision.llAprilTagRear, Constants.Vision.llAprilTagPipelineIndex);

        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
    }

    @Override
    public boolean isFinished() {
        if (Robot.isSimulation()) {
            return false;
        }

        // if (note) {
        //     return false;
        // }

        return !Intake.getInstance().getHolding();
    }
}
