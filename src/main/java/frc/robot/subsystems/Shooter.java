package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LoggyThings.LoggyTalonFX;

import java.util.List;

public class Shooter extends SubsystemBase {
    private static Shooter mInstance;
    /** shootMotorRight is the master motor */
    private LoggyTalonFX shootMotorRight, shootMotorLeft, feedMotor;
    private VelocityVoltage shootPid = new VelocityVoltage(0);

    public static Shooter getInstance() {
        if (mInstance == null) {
            mInstance = new Shooter();
        }

        return mInstance;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/Actual Left RPM", shootMotorLeft.getVelocity().getValueAsDouble() * 60);
        SmartDashboard.putNumber("Shooter/Actual Right RPM", shootMotorRight.getVelocity().getValueAsDouble() * 60);
    }

    private Shooter() {
        shootMotorRight = new LoggyTalonFX(Constants.ShooterConstants.shootMotorRight, false);
        shootMotorLeft = new LoggyTalonFX(Constants.ShooterConstants.shootMotorLeft, false);
        feedMotor = new LoggyTalonFX(Constants.ShooterConstants.feedMotor, false);

        TalonFXConfiguration fxConfig = new TalonFXConfiguration();
        fxConfig.CurrentLimits.SupplyCurrentLimit = 30;
        fxConfig.CurrentLimits.SupplyCurrentThreshold = 60;
        fxConfig.CurrentLimits.SupplyTimeThreshold = 0.5;
        fxConfig.MotorOutput.PeakReverseDutyCycle = 0;

        fxConfig.Slot0.kP = 0.07;
        fxConfig.Slot0.kI = 0.01;
        fxConfig.Slot0.kV = 10.5 / 88.9;
        fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        fxConfig.Audio.AllowMusicDurDisable = true;
        shootMotorLeft.getConfigurator().apply(fxConfig);
        shootMotorRight.getConfigurator().apply(fxConfig);

        fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        fxConfig.Audio.AllowMusicDurDisable = true;
        fxConfig.MotorOutput.PeakReverseDutyCycle = -1;
        feedMotor.getConfigurator().apply(fxConfig);

        feedMotor.setInverted(Constants.ShooterConstants.feedIsInverted);
        shootMotorRight.setInverted(Constants.ShooterConstants.rightShootIsInverted);
        shootMotorLeft.setInverted(Constants.ShooterConstants.leftShootIsInverted);

        Music.getInstance().addFalcon(List.of(shootMotorLeft, shootMotorRight,
                feedMotor));
        // SmartDashboard.putString("Shooter/.type", "Subsystem");
        SmartDashboard.putString("Shooter/Status", "Idle");
    }

    public void setShoot(double feeder, double shooter) {
        setShoot(feeder, shooter, shooter);
    }

    public void setShoot(double feeder, double leftShooter, double rightShooter) {
        shootMotorLeft.set(leftShooter);
        shootMotorRight.set(rightShooter);
        feedMotor.set(feeder);
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
    public void shoot(double feederSetVal, double leftShooterSetVal, double rightShooterSetVal) {
        shootMotorRight
                .setControl(shootPid
                        .withVelocity((Constants.MotorConstants.falconShooterLoadRPM * rightShooterSetVal) / 60));
        shootMotorLeft
                .setControl(shootPid
                        .withVelocity((Constants.MotorConstants.falconShooterLoadRPM * leftShooterSetVal) / 60));
        feedMotor.set(feederSetVal);
    }

    public void stop() {
        shootMotorRight.set(0);
        shootMotorLeft.set(0);
        feedMotor.set(0);
    }

    public void setShooterBrake(boolean brake) {
        NeutralModeValue mode = NeutralModeValue.Coast;
        if (brake) {
            mode = NeutralModeValue.Brake;
        }

        shootMotorRight.setNeutralMode(mode);
        shootMotorLeft.setNeutralMode(mode);
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
        double threshold = (shootMotorLeft.getSupplyVoltage().getValueAsDouble() / 12.8)
                * Constants.MotorConstants.falconShooterLoadRPM;
        SmartDashboard.putNumber("Shooter/Ready To Shoot threshold", threshold);
        return ((shootMotorLeft.getVelocity().getValueAsDouble() * 60) > threshold);
    }

    /**
     * Returns true if the shooter motors are fast enough to shoot, this function
     * checks the left motor and is used when the shooter isnt set to 100%
     * @param setVal the value that the motor was set to
     */
    public boolean readyToShoot(double setVal) {
        double threshold = (shootMotorLeft.getSupplyVoltage().getValueAsDouble() / 12.8)
                * Constants.MotorConstants.falconShooterLoadRPM * setVal;
        SmartDashboard.putNumber("Shooter/Ready To Shoot threshold", threshold);
        return ((shootMotorLeft.getVelocity().getValueAsDouble() * 60) > threshold);
    }
}
