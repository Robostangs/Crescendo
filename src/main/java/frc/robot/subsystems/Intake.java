package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LoggyThings.LoggyTalonFX;

public class Intake extends SubsystemBase {
    public static Intake mInstance;
    public Compressor mCompressor;
    private DoubleSolenoid mSolenoid;

    
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

    public Intake() {
        mSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
                Constants.IntakeConstants.intakeSolenoidFwdID, Constants.IntakeConstants.intakeSolenoidRevID);

        intakeMotor = new LoggyTalonFX(Constants.IntakeConstants.intakeMotorID, false);
        mCompressor = new Compressor(PneumaticsModuleType.CTREPCM);
        mCompressor.enableDigital();
    }

    public void setExtend(boolean extended) {
        if (extended)
            mSolenoid.set(DoubleSolenoid.Value.kForward);
        else
            mSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void setSpin(double speed) {
        intakeMotor.set(speed);
    }

    /* TODO: Should I put belt stuff into here or make a new subsystem? */
}
