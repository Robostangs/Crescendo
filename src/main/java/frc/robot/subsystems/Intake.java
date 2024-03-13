package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Vision.LimelightHelpers;
import frc.robot.subsystems.Drivetrain.Drivetrain;

public class Intake extends SubsystemBase {
    private Solenoid solenoid;
    private TalonFX intakeMotor, beltMotor;
    private DigitalInput shooterSensor;
    private boolean holding = true, setExtend = false;
    
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Shooter/Shooter Sensor", getShooterSensor());
        SmartDashboard.putBoolean("Intake/Holding", holding);

        if (getShooterSensor() && DriverStation.isEnabled()) {
            setHolding(true);

            // LEDs will blink when the arm is at the right setpoint to score
            if (Shooter.getInstance().readyToShootAdvanced() && Drivetrain.getInstance().readyToShoot()) {
                LimelightHelpers.setLEDMode_ForceBlink(Constants.Vision.llAprilTag);
                LimelightHelpers.setLEDMode_ForceBlink(Constants.Vision.llAprilTagRear);
            }

            // LEDs will be on when the arm is not at the right setpoint to score, but the shooter is occupied
            else {
                LimelightHelpers.setLEDMode_ForceOn(Constants.Vision.llAprilTag);
                LimelightHelpers.setLEDMode_ForceOn(Constants.Vision.llAprilTagRear);
            }
        }

        // LEDs will be off when the shooter is not occupied or robot is off
        else {
            LimelightHelpers.setLEDMode_ForceOff(Constants.Vision.llAprilTag);
            LimelightHelpers.setLEDMode_ForceOff(Constants.Vision.llAprilTagRear);
        }
    }
    
    private Intake() {
        solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.IntakeConstants.solenoidID);
        shooterSensor = new DigitalInput(Constants.IntakeConstants.shooterSensorPWM_ID);
        
        intakeMotor = new TalonFX(Constants.IntakeConstants.shooterMotorID, "*");
        beltMotor = new TalonFX(Constants.IntakeConstants.beltMotorID, "*");
        
        intakeMotor.setInverted(Constants.IntakeConstants.intakeMotorInverted);
        beltMotor.setInverted(Constants.IntakeConstants.beltMotorInverted);

        holding = false;

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
