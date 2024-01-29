package frc.robot.commands.intake;

import java.util.LinkedList;
import java.util.Queue;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class NoteAlign extends Command {
    private CommandSwerveDrivetrain mDrivetrain = CommandSwerveDrivetrain.getInstance();
    private SwerveRequest.RobotCentric drive;
    // private PIDController mPID = new PIDController(0.1, 0.15, 0);
    private PIDController mPID = new PIDController(IntakeConstants.ALIGN_P, IntakeConstants.ALIGN_I, IntakeConstants.ALIGN_D);
    private DoubleSupplier leftX;
    private double xSpeed, MaxSpeed;

    private final NetworkTableEntry tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx");
    private final int positionsRecorded = 10; // How many past positions will be included in the position average
    Queue<Double> noteHist;
    private double runningSum;
    
    public NoteAlign(DoubleSupplier leftX) {
        addRequirements(mDrivetrain);

        // TODO: Make this robot centric where it drives forward automatically
        drive = new SwerveRequest.RobotCentric()
                    .withDeadband(DrivetrainConstants.MAX_SPEED * 0.08)
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        this.leftX = leftX;
    }

    public double getNoteX() {
        double x = tx.getDouble(0);
        noteHist.add(x);
        runningSum += x;
        if (noteHist.size()>positionsRecorded){
            runningSum -= noteHist.remove();
        }   
        return runningSum/noteHist.size();
    }

    @Override
    public void initialize() {
        noteHist = new LinkedList<>();
    }

    @Override
    public void execute() {
        xSpeed = leftX.getAsDouble();
        mDrivetrain.setControl(drive.withVelocityY(IntakeConstants.DRIVE_SPEED) // Drive forward with negative Y (forward)
            .withVelocityX(-xSpeed * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(mPID.calculate(getNoteX()))// Drive counterclockwise with negative X (left)
        );
    }
}
