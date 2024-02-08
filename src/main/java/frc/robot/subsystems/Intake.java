package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Intake extends SubsystemBase {
    
    public static Intake mIntake;
    public Compressor mCompressor;
    private DoubleSolenoid mSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.IntakeConstants.intakeSolenoidFwdID, Constants.IntakeConstants.intakeSolenoidRevID); // TODO: Make sure this is right
    private TalonFX intakeMotor = new TalonFX(Constants.IntakeConstants.intakeMotorID, "rio");

    public static Intake getInstance() {
        if (mIntake == null)
            mIntake = new Intake();
        return mIntake;
    }

    public Intake() {
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
}
