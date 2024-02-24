package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LoggyThings.LoggyTalonFX;
import frc.robot.Vision.LimelightHelpers;

public class Intake extends SubsystemBase {
    private static Intake mInstance;
    private Compressor mCompressor;
    private Solenoid solenoid;
    private LoggyTalonFX intakeMotor, beltMotor;
    private DigitalInput shooterSensor;
    private boolean holding = true;

    @Override
    public void periodic() {
        SmartDashboard.putString("Intake/Status", solenoid.get() ? "Extended" : "Retracted");
        SmartDashboard.putBoolean("Shooter/Shooter Sensor", getShooterSensor());
        SmartDashboard.putBoolean("Intake/Holding", holding);

        if (getShooterSensor() && DriverStation.isEnabled()) {
            LimelightHelpers.setLEDMode_ForceBlink(Constants.Vision.llAprilTag);
            LimelightHelpers.setLEDMode_ForceBlink(Constants.Vision.llAprilTagRear);          
        } else {
            LimelightHelpers.setLEDMode_ForceOff(Constants.Vision.llAprilTag);
            LimelightHelpers.setLEDMode_ForceOff(Constants.Vision.llAprilTagRear);
        }
    }

    public static Intake getInstance() {
        if (mInstance == null)
            mInstance = new Intake();
        return mInstance;
    }

    private Intake() {
        solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.IntakeConstants.solenoidID);
        mCompressor = new Compressor(PneumaticsModuleType.CTREPCM);
        mCompressor.enableDigital();
        // mCompressor.disable();
        shooterSensor = new DigitalInput(Constants.IntakeConstants.shooterSensorPWM_ID);

        intakeMotor = new LoggyTalonFX(Constants.IntakeConstants.shooterMotorID, true);
        beltMotor = new LoggyTalonFX(Constants.IntakeConstants.beltMotorID, true);

        intakeMotor.setInverted(Constants.IntakeConstants.intakeMotorInverted);
        beltMotor.setInverted(Constants.IntakeConstants.beltMotorInverted);
    }

    public void setExtend(boolean deploy) {
        solenoid.set(deploy);
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

    public void stop() {
        intakeMotor.set(0);
        beltMotor.set(0);
    }
}
