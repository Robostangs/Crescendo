package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LoggyThings.LoggyTalonFX;

public class Intake extends SubsystemBase {
    private static Intake mInstance;
    private Compressor mCompressor;
    private Solenoid mSolenoid;
    // private DoubleSolenoid mSolenoid;

    
    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
    }
    
    private LoggyTalonFX intakeMotor;
    
    public static Intake getInstance() {
        if (mInstance == null)
            mInstance = new Intake();
        return mInstance;
    }

    private Intake() {
        mSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
        // mSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
        //         Constants.IntakeConstants.intakeSolenoidFwdID, Constants.IntakeConstants.intakeSolenoidRevID);

        intakeMotor = new LoggyTalonFX(Constants.IntakeConstants.intakeMotorID, false);
        mCompressor = new Compressor(PneumaticsModuleType.CTREPCM);
        mCompressor.enableDigital();
    }

    public void setExtend(boolean extended) {
        if (extended) {
            mSolenoid.set(true);
            // mSolenoid.set(DoubleSolenoid.Value.kForward);
        } else {
            // mSolenoid.set(DoubleSolenoid.Value.kReverse);
        }
    }

    public void setMotor(double speed) {
        intakeMotor.set(speed);
    }

    /* TODO: Should I put belt stuff into here or make a new subsystem? */
}
