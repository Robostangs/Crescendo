package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private TalonFX bottomShooter, topShooter, feedMotor;
    private VelocityVoltage shootPid = new VelocityVoltage(0);

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/Actual Top Motor RPM", topShooter.getVelocity().getValueAsDouble() * 60);
        SmartDashboard.putNumber("Shooter/Actual Bottom Motor RPM", bottomShooter.getVelocity().getValueAsDouble() * 60);
    }

    private Shooter() {
        bottomShooter = new TalonFX(Constants.ShooterConstants.bottomShooterMotorID, "rio");
        topShooter = new TalonFX(Constants.ShooterConstants.topShooterMotorID, "rio");
        feedMotor = new TalonFX(Constants.ShooterConstants.feedMotor, "rio");

        TalonFXConfiguration fxConfig = new TalonFXConfiguration();
        fxConfig.CurrentLimits.SupplyCurrentLimit = 30;
        fxConfig.CurrentLimits.SupplyCurrentThreshold = 60;
        fxConfig.CurrentLimits.SupplyTimeThreshold = 0.5;

        fxConfig.Slot0.kP = 0.1;
        fxConfig.Slot0.kI = 0.01;
        fxConfig.Slot0.kV = 10.5 / 88.9;
        fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        fxConfig.Audio.AllowMusicDurDisable = true;
        topShooter.getConfigurator().apply(fxConfig);
        bottomShooter.getConfigurator().apply(fxConfig);

        fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        fxConfig.Audio.AllowMusicDurDisable = true;
        feedMotor.getConfigurator().apply(fxConfig);

        feedMotor.setInverted(Constants.ShooterConstants.feedIsInverted);
        bottomShooter.setInverted(Constants.ShooterConstants.rightShootIsInverted);
        topShooter.setInverted(Constants.ShooterConstants.leftShootIsInverted);

        SmartDashboard.putString("Shooter/Status", "Idle");

        Music.getInstance().addFalcon(topShooter, bottomShooter, feedMotor);
    }

    public void setShoot(double feederDutyCycle, double shooterDutyCycle) {
        setShoot(feederDutyCycle, shooterDutyCycle, shooterDutyCycle);
    }

    public void setShoot(double feederDutyCycle, double topShooterDutyCycle, double bottomShooterDutyCycle) {
        topShooter.set(topShooterDutyCycle);
        bottomShooter.set(bottomShooterDutyCycle);
        feedMotor.set(feederDutyCycle);
    }

    /**
     * Velocity is in RPM, values should be [-1,1]
     * <p>
     * feedMotor does not use Velocity PID
     * 
     */
    public void shoot(double feederSetValue, double shooterSetValue) {
        shoot(feederSetValue, shooterSetValue, shooterSetValue);
    }

    /**
     * Velocity is in RPM, values should be [-1,1]
     * <p>
     * feedMotor does not use Velocity PID
     */
    public void shoot(double feederDutyCycle, double topShooterSetVal, double bottomShooterSetVal) {
        
        // dont use PID to get to 0rpm
        if (bottomShooterSetVal == 0d) {
            bottomShooter.set(0);
        }

        else {
            bottomShooter
                    .setControl(shootPid
                            .withVelocity((Constants.MotorConstants.falconShooterLoadRPM * bottomShooterSetVal) / 60));
        }

        // dont use PID to get to 0rpm
        if (topShooterSetVal == 0d) {
            topShooter.set(0);
        }

        else {
            topShooter
                    .setControl(shootPid
                            .withVelocity((Constants.MotorConstants.falconShooterLoadRPM * topShooterSetVal) / 60));
        }

        // feedMotor does not use Velocity PID
        feedMotor.set(feederDutyCycle);
    }

    public void stop() {
        bottomShooter.set(0);
        topShooter.set(0);
        feedMotor.set(0);
    }

    public void setShooterBrake(boolean brake) {
        NeutralModeValue mode = NeutralModeValue.Coast;

        if (brake) {
            mode = NeutralModeValue.Brake;
        }

        bottomShooter.setNeutralMode(mode);
        topShooter.setNeutralMode(mode);
    }

    public void setFeederBrake(boolean brake) {
        NeutralModeValue mode = NeutralModeValue.Coast;
        if (brake) {
            mode = NeutralModeValue.Brake;
        }

        feedMotor.setNeutralMode(mode);
    }

    /**
     * Returns true if the shooter motors are fast enough to shoot, this function
     * checks the left motor
     */
    public boolean readyToShoot() {
        return readyToShoot(1);
    }

    /**
     * Returns true if the shooter motors are fast enough to shoot, this function
     * checks the left motor and is used when the shooter isnt set to 100%
     * 
     * @param setVal the value that the motor was set to
     */
    public boolean readyToShoot(double setVal) {
        double threshold;

        threshold = Constants.MotorConstants.falconShooterThresholdRPM * setVal;

        SmartDashboard.putNumber("Shooter/Ready To Shoot threshold", threshold);
        return ((topShooter.getVelocity().getValueAsDouble() * 60) > threshold && (bottomShooter.getVelocity()
                .getValueAsDouble() * 60) > threshold);
    }

    /**
     * If the shooter is at the right angle, the shooter wheels are spinning fast
     * enough, and the robot is aligned with the target and not currently rotating
     * much
     * 
     * @return returns true if ready to feed (and shoot) right now
     */
    public boolean readyToShootAdvanced() {
        return Arm.getInstance().isInRangeOfTarget(Arm.getInstance().calculateArmSetpoint()) &&
                Math.abs(Arm.getInstance().getVelocity()) < 0.25 && readyToShoot();
    }

    private static Shooter mInstance;

    public static Shooter getInstance() {
        if (mInstance == null) {
            mInstance = new Shooter();
        }

        return mInstance;
    }
}
