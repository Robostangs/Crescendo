/* TODO: this dont work cuz i changed a bunch of stuff, rewrite this cuz might be useful */

package frc.robot.commands.Swerve;

import java.util.LinkedList;
import java.util.Queue;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Vision.LimelightHelpers;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.subsystems.Drivetrain.SwerveRequest;
import frc.robot.subsystems.Drivetrain.SwerveModule.DriveRequestType;

public class Align extends Command {
    private Drivetrain mDrivetrain = Drivetrain.getInstance();
    private SwerveRequest.FieldCentricFacingAngle drive;

    // private PIDController mPID = new PIDController(
    //         Constants.AutoConstants.noteAlignPID.kP,
    //         Constants.AutoConstants.noteAlignPID.kI,
    //         Constants.AutoConstants.noteAlignPID.kD);

    private DoubleSupplier xTranslation, yTranslation, babiesOnBoard;
    private DoubleSupplier getTargetX;
    private boolean note;

    private final int positionsRecorded = 10;
    private Queue<Double> noteHist;
    private double runningSum;

    /** Test this soon af */
    public Align(DoubleSupplier xTranslation, DoubleSupplier yTranslation, DoubleSupplier babiesOnBoard, boolean note) {
        this.addRequirements(mDrivetrain);
        this.setName("Align");

        drive = new SwerveRequest.FieldCentricFacingAngle()
                .withDeadband(OperatorConstants.deadband)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        this.xTranslation = xTranslation;
        this.yTranslation = yTranslation;
        this.babiesOnBoard = babiesOnBoard;
        this.note = note;


        if (note) {
            getTargetX = () -> LimelightHelpers.getTX(Constants.Vision.llPython);
        } else {
            getTargetX = () -> LimelightHelpers.getTX(Constants.Vision.llAprilTagRear);
        }
    }

    public double getNoteX() {
        double x = getTargetX.getAsDouble();
        noteHist.add(x);
        runningSum += x;
        if (noteHist.size() > positionsRecorded) {
            runningSum -= noteHist.remove();
        }
        return runningSum / noteHist.size();
    }

    @Override
    public void initialize() {
        noteHist = new LinkedList<>();
        if (note) {
            LimelightHelpers.setPipelineIndex(Constants.Vision.llPython, Constants.Vision.llPythonPipelineIndex);
        } else {
            LimelightHelpers.setPipelineIndex(Constants.Vision.llAprilTagRear, 1);
        }

        /* TODO: i heavily doubt this works, the problem is that getTargetX will return a rotation that is relative to robot, not to X axis */
        drive.TargetDirection = Rotation2d.fromDegrees(getTargetX.getAsDouble());
    }

    @Override
    public void execute() {
        mDrivetrain.setControl(
                drive
                        .withVelocityX(xTranslation.getAsDouble() * Constants.SwerveConstants.kMaxSpeedMetersPerSecond)
                        .withVelocityY(
                                yTranslation.getAsDouble() * Constants.SwerveConstants.kMaxSpeedMetersPerSecond)
                        .withSlowDown(true, 1 - babiesOnBoard.getAsDouble()));

        // xSpeed = leftX.getAsDouble();
        // mDrivetrain.setControl(drive.withVelocityY(Constants.AutoConstants.driveSpeed)
        // // Drive forward with negative Y
        // // (forward)
        // .withVelocityX(-xSpeed * MaxSpeed) // Drive left with negative X (left)
        // .withRotationalRate(mPID.calculate(getNoteX()))// Drive counterclockwise with
        // negative X (left)
        // );
    }
}
