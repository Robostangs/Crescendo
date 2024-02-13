package frc.robot.commands.Swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Vision.LimelightHelpers;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.subsystems.Drivetrain.SwerveRequest;

public class Align extends Command {
    private Drivetrain mDrivetrain = Drivetrain.getInstance();
    private Intake mIntake = Intake.getInstance();

    private SwerveRequest drive;
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

        if (this.note) {
            getTargetRotation = () -> Rotation2d.fromDegrees(LimelightHelpers.getTX(Constants.Vision.llPython));
            this.addRequirements(mIntake);
        } else {
            getTargetRotation = () -> {
                Pose2d pose = mDrivetrain.getPose();
                double rotation = Math.atan2(Constants.Vision.SpeakerCoords[1] - pose.getY(),
                        Constants.Vision.SpeakerCoords[0]);
                return Rotation2d.fromRadians(rotation);
            };
            // getTargetX = () -> LimelightHelpers.getTX(Constants.Vision.llAprilTagRear);
        }
    }

    @Override
    public void initialize() {
        timer.restart();
        if (note) {
            LimelightHelpers.setPipelineIndex(Constants.Vision.llPython, Constants.Vision.llPythonPipelineIndex);
        } else {
            LimelightHelpers.setPipelineIndex(Constants.Vision.llAprilTagRear, 1);
        }
    }

    @Override
    public void execute() {
        if (note) {
            mIntake.setExtend(true);
            if (timer.get() > Constants.IntakeConstants.kDeployTimeSeconds) {
                mIntake.setIntake(1);
            }
        }

        if (Math.abs(translateX.get()) <= Constants.OperatorConstants.kDeadzone
                && Math.abs(translateY.get()) <= Constants.OperatorConstants.kDeadzone) {
            drive = new SwerveRequest.SwerveDriveBrake();
        } else {
            drive = new SwerveRequest.FieldCentricFacingAngle()
                    .withVelocityX(-translateY.get()
                            * Constants.SwerveConstants.kMaxSpeedMetersPerSecond)
                    .withVelocityY(-translateX.get()
                            * Constants.SwerveConstants.kMaxSpeedMetersPerSecond)
                    /*
                     * TODO: i heavily doubt this works, the problem is that getTargetRotation will return
                     * a rotation that is relative to robot, not to X axis if the note boolean is true
                     */
                    .withTargetDirection(getTargetRotation.get())
                    .withSlowDown(true, 1 - howManyBabiesOnBoard.get())
                    .withRotationalDeadband(Constants.OperatorConstants.rotationalDeadband)
                    .withDeadband(Constants.OperatorConstants.deadband);
        }

        mDrivetrain.setControl(drive);
    }

    @Override
    public void end(boolean interrupted) {
        mDrivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
        if (note) {
            mIntake.setIntake(0);
            mIntake.setExtend(false);
        }
    }
}
