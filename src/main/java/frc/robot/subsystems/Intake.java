package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LoggyThings.LoggyTalonFX;

public class Intake extends SubsystemBase {
    private static Intake mInstance;
    private Compressor mCompressor;
    private Solenoid leftSolenoid, rightSolenoid;
    private LoggyTalonFX intakeMotor, beltMotor;
    private DigitalInput intakeSensor, beltSensor;

    @Override
    public void periodic() {
        if (leftSolenoid.get() != rightSolenoid.get()) {
            SmartDashboard.putString("Intake/Status", "ERROR: Solenoids are not in sync");
            // setExtend(false);
        }

        SmartDashboard.putString("Intake/Status", leftSolenoid.get() ? "Extended" : "Retracted");

        if (getExtended() && !getBeltSensor()) {
            beltMotor.set(1);
        }
    }

    public static Intake getInstance() {
        if (mInstance == null)
            mInstance = new Intake();
        return mInstance;
    }

    private Intake() {
        leftSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.IntakeConstants.leftSolenoidID);
        rightSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.IntakeConstants.rightSolenoidID);
        mCompressor = new Compressor(PneumaticsModuleType.CTREPCM);
        intakeSensor = new DigitalInput(Constants.IntakeConstants.intakeSensorPWM_ID);
        beltSensor = new DigitalInput(Constants.IntakeConstants.beltSensorPWM_ID);
        mCompressor.enableDigital();
        mCompressor.disable();

        intakeMotor = new LoggyTalonFX(Constants.IntakeConstants.intakeMotorID, true);
        // beltMotor = new LoggyTalonFX(Constants.IntakeConstants.beltMotorID, true);

        TalonFXConfiguration fxConfig = new TalonFXConfiguration();
        fxConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        intakeMotor.getConfigurator().apply(fxConfig);
        // beltMotor.getConfigurator().apply(fxConfig);
    }

    public void setExtend(boolean deploy) {
        leftSolenoid.set(deploy);
        rightSolenoid.set(deploy);
    }

    public void setMotor(double speed) {
        intakeMotor.set(speed);
    }

    public boolean getIntakeSensor() {
        return intakeSensor.get();
    }

    public boolean getBeltSensor() {
        return beltSensor.get();
    }

    public void setBelt(double speed) {
        beltMotor.set(speed);
    }

    public boolean getExtended() {
        return leftSolenoid.get();
    }
}
