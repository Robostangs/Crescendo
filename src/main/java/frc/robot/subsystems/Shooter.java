package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
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

    private DigitalInput ringSensor;

    private boolean holding;

    public static Shooter getInstance() {
        if (mInstance == null) {
            mInstance = new Shooter();
        }

        return mInstance;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Shooter/Holding", holding);
    }

    private Shooter() {
        holding = true;

        shootMotorRight = new LoggyTalonFX(Constants.ShooterConstants.shootMotorRight, false);
        shootMotorLeft = new LoggyTalonFX(Constants.ShooterConstants.shootMotorLeft, false);
        feedMotor = new LoggyTalonFX(Constants.ShooterConstants.feedMotor, false);
        ringSensor = new DigitalInput(0);

        TalonFXConfiguration fxConfig = new TalonFXConfiguration();
        fxConfig.CurrentLimits.SupplyCurrentLimit = 30;
        fxConfig.CurrentLimits.SupplyCurrentThreshold = 60;
        fxConfig.CurrentLimits.SupplyTimeThreshold = 0.5;
        fxConfig.MotorOutput.PeakReverseDutyCycle = 0;
        fxConfig.Slot0.kP = 0.2;
        fxConfig.Slot0.kI = 0.07;
        fxConfig.Slot0.kV = 2 / 16;
        fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        fxConfig.Feedback.SensorToMechanismRatio = 14 / 20;
        fxConfig.Audio.AllowMusicDurDisable = true;
        shootMotorLeft.getConfigurator().apply(fxConfig);
        shootMotorRight.getConfigurator().apply(fxConfig);
        feedMotor.getConfigurator().apply(fxConfig);

        feedMotor.setInverted(Constants.ShooterConstants.feedIsInverted);
        shootMotorRight.setInverted(Constants.ShooterConstants.rightShootIsInverted);
        shootMotorLeft.setInverted(Constants.ShooterConstants.leftShootIsInverted);

        Music.getInstance().addFalcon(List.of(shootMotorLeft, shootMotorRight,
                feedMotor));
        SmartDashboard.putString("Shooter/.type", "Subsystem");
        SmartDashboard.putString("Shooter/Status", "Idle");
        SmartDashboard.putBoolean("Shooter/Loaded", getHolding());

    }

    public void shoot(double feeder, double shooter) {
        shootMotorRight.set(shooter);
        shootMotorLeft.set(shooter);
        feedMotor.set(feeder);
    }

    public void shoot(double feeder, double leftShooter, double rightShooter) {
        feedMotor.set(feeder);
        shootMotorLeft.set(leftShooter);
        shootMotorRight.set(rightShooter);
    }

    public void stop() {
        shootMotorRight.set(0);
        shootMotorLeft.set(0);
        feedMotor.set(0);
    }

    // public void setHolding(boolean holding) {
    //     this.holding = holding;
    // }

    // public void toggleHolding() {
    //     holding = !holding;
    // }

    public boolean getHolding() {
        return ringSensor.get();
    }

    /**
     * @deprecated idk why we would ever use this
     * @return whether or not the shoot motor right is below the threshold RPM (1000
     *         RPM)
     */
    public boolean getSpeedChange() {
        if (shootMotorRight.getVelocity().getValueAsDouble() < Constants.MotorConstants.FalconRotorLoadThresholdRPM) {
            return true;
        }
        return false;
    }

    public void SetRpm(double left, double right) {
        /* TODO: Are we even going to use this? (shootPid) */
        shootMotorRight.setControl(shootPid.withVelocity(right / 60));
        shootMotorLeft.setControl(shootPid.withVelocity(left / 60));
    }

    public void setBrakeMode(boolean brake) {
        NeutralModeValue mode = NeutralModeValue.Coast;
        if (brake) {
            mode = NeutralModeValue.Brake;
        }

        shootMotorRight.setNeutralMode(mode);
        shootMotorLeft.setNeutralMode(mode);
    }
}