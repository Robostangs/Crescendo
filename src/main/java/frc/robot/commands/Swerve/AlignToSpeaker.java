package frc.robot.commands.Swerve;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.subsystems.Drivetrain.SwerveRequest;

public class AlignToSpeaker extends Command {
    Drivetrain drivetrain;

    SwerveRequest.FieldCentricFacingAngle driveRequest;

    Supplier<Double> translateX, translateY, howManyBabiesOnBoard;
    Supplier<Rotation2d> getTargetRotation;

    public AlignToSpeaker() {
        this(() -> 0.0, () -> 0.0, () -> 0.0);
    }

    /**
     * Command to set the drivetrain to brake mode when not moving
     * 
     * @param translateX           the forward to backward movement of the robot
     * @param translateY           the right to left movement of the robot
     * @param howManyBabiesOnBoard 1 - the value of how much to slow down (right
     *                             trigger axis)
     */
    public AlignToSpeaker(Supplier<Double> translateX, Supplier<Double> translateY,
            Supplier<Double> howManyBabiesOnBoard) {

        drivetrain = Drivetrain.getInstance();

        this.addRequirements(drivetrain);

        if (howManyBabiesOnBoard == null) {
            this.howManyBabiesOnBoard = () -> 0.0;
        }

        else {
            this.howManyBabiesOnBoard = howManyBabiesOnBoard;
        }

        this.translateX = translateX;
        this.translateY = translateY;

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

    @Override
    public void initialize() {
        driveRequest = new SwerveRequest.FieldCentricFacingAngle();
        driveRequest.HeadingController = new PhoenixPIDController(12, 12, 1);

        // this is for tuning and now we can tune the PID controller
        SmartDashboard.putData("Align to Speaker PID", driveRequest.HeadingController);
        drivetrain.postStatus("Aligning to Speaker");

        driveRequest.Deadband = Constants.OperatorConstants.deadband;
        driveRequest.RotationalDeadband = Constants.OperatorConstants.rotationalDeadband * 0.05;
    }

    @Override
    public void execute() {
        driveRequest.TargetDirection = getTargetRotation.get();

        driveRequest
                .withVelocityX(-translateX.get()
                        * Constants.SwerveConstants.kMaxSpeedMetersPerSecond)
                .withVelocityY(translateY.get()
                        * Constants.SwerveConstants.kMaxSpeedMetersPerSecond)
                .withSlowDown(1 - howManyBabiesOnBoard.get());

        drivetrain.setControl(driveRequest);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
        drivetrain.postStatus("Aligned");
    }

    @Override
    public boolean isFinished() {
        if (Robot.isSimulation()) {
            return false;
        }

        else {
            return !Intake.getInstance().getHolding();
        }

    }
}