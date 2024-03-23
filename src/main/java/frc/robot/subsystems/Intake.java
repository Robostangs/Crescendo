package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Intake extends SubsystemBase {
    private Solenoid solenoid;
    private Compressor compressor;
    private TalonFX intakeMotor, beltMotor;
    private DigitalInput shooterSensor;
    private boolean holding, setExtend;
    
    @Override
    public void periodic() {
        if (getShooterSensor()) {
            setHolding(true);
        }
        SmartDashboard.putBoolean("Intake/Shooter Sensor", getShooterSensor());
        SmartDashboard.putBoolean("Intake/Holding", holding);
    }
    
    private Intake() {
        solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.IntakeConstants.solenoidID);
        shooterSensor = new DigitalInput(Constants.IntakeConstants.shooterSensorPWM_ID);

        compressor = new Compressor(PneumaticsModuleType.CTREPCM);
        compressor.enableDigital();
        
        intakeMotor = new TalonFX(Constants.IntakeConstants.intakeMotorID, "*");
        beltMotor = new TalonFX(Constants.IntakeConstants.beltMotorID, "*");
        
        intakeMotor.setInverted(Constants.IntakeConstants.intakeMotorInverted);
        beltMotor.setInverted(Constants.IntakeConstants.beltMotorInverted);

        holding = false;
        setExtend = false;

        if (Robot.isSimulation()) {
            SmartDashboard.putBoolean("Shooter/Simulated Shooter Sensor", false);
        }
    }

    public void setExtend(boolean deploy) {
        solenoid.set(deploy);
        SmartDashboard.putString("Intake/Crashbar Status", deploy ? "Extended" : "Retracted");
    }

    public void setIntake(double speed) {
        intakeMotor.set(speed);
    }

    public void setBelt(double speed) {
        beltMotor.set(speed);
    }

    public void feedBelt() {
        beltMotor.set(0.5);
    }

    /**
     * Returns true if a piece is inside the shooter, false if the shooter is vacant
     * 
     * @return true = shooter occupied, false = shooter is vacant
     */
    public boolean getShooterSensor() {
        if (Robot.isSimulation()) {
            return SmartDashboard.getBoolean("Shooter/Simulated Shooter Sensor", false);
        }
        
        return !shooterSensor.get();
    }

    public boolean getExtended() {
        return solenoid.get();
    }

    public void setHolding(boolean holding) {
        this.holding = holding;
    }

    public boolean getHolding() {
        return holding;
    }

    /**
     * Stops the intake and belt motors, and retracts the intake
     */
    public void stop() {
        setIntake(0);
        setBelt(0);
        setExtend(false);
    }

    public void setIntakeExtend(boolean extend) {
        setExtend = extend;
        setHolding(!extend);
    }

    public void extendIntakeToggle() {
        setExtend = !setExtend;
        setHolding(setExtend);
    }

    private static Intake mInstance;

    public static Intake getInstance() {
        if (mInstance == null)
            mInstance = new Intake();
        return mInstance;
    }
}
