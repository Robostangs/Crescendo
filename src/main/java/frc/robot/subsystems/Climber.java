// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

    private TalonFX mLeftClimberMotor, mRightClimberMotor;
    private Solenoid mRightBrakeSolenoid, mLeftBrakeSolenoid;
    private StatusSignal<Double> mLeftPosition, mRightPosition;
    private StatusSignal<ReverseLimitValue> mLeftLimitStatus, mRightLimitStatus;

    private TalonFXConfiguration talonConfig = new TalonFXConfiguration();

    private Climber() {
        mLeftClimberMotor = new TalonFX(Constants.ClimberConstants.LeftMotor.kId);
        mLeftClimberMotor.setInverted(Constants.ClimberConstants.LeftMotor.kInverted);
        mRightClimberMotor = new TalonFX(Constants.ClimberConstants.RightMotor.kId);
        mRightClimberMotor.setInverted(Constants.ClimberConstants.RightMotor.kInverted);

        talonConfig.Feedback.SensorToMechanismRatio = Constants.ClimberConstants.LeftMotor.kGearboxRotationsToMechanismMeters;
        talonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        talonConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.ClimberConstants.LeftMotor.kMaxExtensionMeters;
        talonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        //CHANGE THE LIMIT OVERRIDES IF WE DO THIS TOOOOOOOOO
        //talonConfig.HardwareLimitSwitch.ReverseLimitEnable = true;
        //talonConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
        talonConfig.CurrentLimits.StatorCurrentLimit = Constants.ClimberConstants.kDefaultStatorCurrentLimit;
        talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        mLeftClimberMotor.getConfigurator().apply(talonConfig);
        mRightClimberMotor.getConfigurator().apply(talonConfig);

        mLeftClimberMotor.setPosition(0);
        mRightClimberMotor.setPosition(0);

        mLeftBrakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM,
                Constants.ClimberConstants.LeftBrakeSolenoid.kId);
        mRightBrakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM,
                Constants.ClimberConstants.RightBrakeSolenoid.kId);

        setDefaultCommand(new RunCommand(() -> {
            setLeftClimbPower(0);
            setRightClimbPower(0);
        }, this));

        mLeftPosition = mLeftClimberMotor.getPosition();
        mRightPosition = mRightClimberMotor.getPosition();
        mLeftLimitStatus = mLeftClimberMotor.getReverseLimit();
        mRightLimitStatus = mRightClimberMotor.getReverseLimit();

    }

    public void setLeftClimbPower(double power) {
        boolean solenoidVal = (power != 0);
        mLeftBrakeSolenoid.set(solenoidVal);
        SmartDashboard.putBoolean("Climber/Left/Brake Solenoid", solenoidVal);
        SmartDashboard.putNumber("Climber/Left/Power", power);
        mLeftClimberMotor.set(power);
    }

    public void setRightClimbPower(double power) {
        boolean solenoidVal = (power != 0);
        mRightBrakeSolenoid.set(solenoidVal);
        SmartDashboard.putBoolean("Climber/Right/Brake Solenoid", solenoidVal);
        SmartDashboard.putNumber("Climber/Right/Power", power);
        mRightClimberMotor.set(power);
    }

    public double getLeftPosition() {
        return mLeftPosition.getValueAsDouble();
    }

    public double getRightPosition() {
        return mRightPosition.getValueAsDouble();
    }

    public void setLeftPosition(double position) {
        mLeftClimberMotor.setPosition(position);
    }

    public void setRightPosition(double position) {
        mRightClimberMotor.setPosition(position);
    }

    public boolean isLeftLimitSwitchHit() {
        return mLeftLimitStatus.getValue() == ReverseLimitValue.ClosedToGround;
    }

    public boolean isRightLimitSwitchHit() {
        return mRightLimitStatus.getValue() == ReverseLimitValue.ClosedToGround;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        mLeftPosition.refresh();
        mRightPosition.refresh();

        mLeftLimitStatus.refresh();
        mRightLimitStatus.refresh();

        SmartDashboard.putNumber("Climber/Left/Motor Position", getLeftPosition());
        SmartDashboard.putBoolean("Climber/Left/Limit Switch", isLeftLimitSwitchHit());

        SmartDashboard.putNumber("Climber/Right/Motor Position", getRightPosition());
        SmartDashboard.putBoolean("Climber/Right/Limit Switch", isRightLimitSwitchHit());
    }

    public void setCurrentLimits(double kCurrentLimit) {
        mLeftClimberMotor.getConfigurator().apply(talonConfig.CurrentLimits.withStatorCurrentLimit(kCurrentLimit));
        mRightClimberMotor.getConfigurator().apply(talonConfig.CurrentLimits.withStatorCurrentLimit(kCurrentLimit));
    }
    public void setReverseSoftLimitState(boolean softLimitEnable) {
        mLeftClimberMotor.getConfigurator().apply(talonConfig.SoftwareLimitSwitch.withReverseSoftLimitEnable(softLimitEnable));
        mRightClimberMotor.getConfigurator().apply(talonConfig.SoftwareLimitSwitch.withReverseSoftLimitEnable(softLimitEnable));
    }

    private static Climber instance;

    public static Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }

        return instance;
    }

    public void setAllLimitOverride(boolean limitOverride){
        mLeftClimberMotor.getConfigurator().apply(talonConfig.SoftwareLimitSwitch.withReverseSoftLimitEnable(!limitOverride).withForwardSoftLimitEnable(!limitOverride));
        mRightClimberMotor.getConfigurator().apply(talonConfig.SoftwareLimitSwitch.withReverseSoftLimitEnable(!limitOverride).withForwardSoftLimitEnable(!limitOverride));
        //mLeftClimberMotor.getConfigurator().apply(talonConfig.HardwareLimitSwitch.withReverseLimitEnable(!limitOverride));
        //mRightClimberMotor.getConfigurator().apply(talonConfig.HardwareLimitSwitch.withReverseLimitEnable(!limitOverride));
    }
}