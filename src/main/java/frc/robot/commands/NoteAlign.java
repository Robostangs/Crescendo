package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class NoteAlign extends Command {
    private CommandSwerveDrivetrain mSwerve;
    private SwerveRequest.FieldCentric drive;
    private PIDController mPID = new PIDController(0.3, 0, 0);
    private DoubleSupplier leftX, leftY;
    private double xSpeed, ySpeed, MaxSpeed;

    private int positionsRecorded = 10; // How many past positions will be included in the position average
    private double[] noteXHistory = new double[10];
    private int historyCounter = 0;
    private double historySum;
    private double historyAverage;
    private double worstPosition;
    
    public NoteAlign(CommandSwerveDrivetrain mSwerve, DoubleSupplier leftX, DoubleSupplier leftY, double MaxSpeed) {
        this.mSwerve = mSwerve;
        addRequirements(mSwerve);

        drive = new SwerveRequest.FieldCentric()
                    .withDeadband(TunerConstants.kSpeedAt12VoltsMps * 0.08)
                    .withRotationalDeadband(0)
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        this.leftX = leftX;
        this.leftY = leftY;
        this.MaxSpeed = MaxSpeed;
    }

    public double getNoteX() {
        noteXHistory[historyCounter] = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        historyCounter = (historyCounter + 1)%positionsRecorded;
        worstPosition = 0;
        historySum = 0;

        for (double i : noteXHistory) 
            historySum += i;

        historyAverage = historySum/positionsRecorded;

        for (double i : noteXHistory)
            if (Math.abs(i-historyAverage) > Math.abs(worstPosition-historyAverage))
                worstPosition = i;

        return (historySum-worstPosition)/(positionsRecorded-1);
    }

    @Override
    public void initialize() {
        noteXHistory = new double[10];
    }

    @Override
    public void execute() {
        xSpeed = leftX.getAsDouble();
        ySpeed = leftY.getAsDouble();
        mSwerve.setControl(drive.withVelocityY(-xSpeed * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityX(-ySpeed * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(mPID.calculate(getNoteX()))// Drive counterclockwise with negative X (left)
        );
    }
}
