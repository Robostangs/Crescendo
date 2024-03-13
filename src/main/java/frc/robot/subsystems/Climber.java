// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
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
    private double mLeftPositionRotations, mRightPositionRotations;
    private boolean mLeftLimitStatus, mRightLimitStatus;
    
    private TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    
    private static Climber instance;
    
    public static Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }

        return instance;
    }

    private Climber() {
        mLeftClimberMotor = new TalonFX(Constants.ClimberConstants.leftClimberMotorID);
        mLeftClimberMotor.setInverted(Constants.ClimberConstants.leftClimberMotorInverted);
        mRightClimberMotor = new TalonFX(Constants.ClimberConstants.rightClimberMotorID);
        mRightClimberMotor.setInverted(Constants.ClimberConstants.rightClimberMotorInverted);

        talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        talonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        talonConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.ClimberConstants.climberMaxExtensionRotations;
        talonConfig.HardwareLimitSwitch.ReverseLimitEnable = true;
        talonConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
        talonConfig.CurrentLimits.StatorCurrentLimit = Constants.ClimberConstants.kDefaultStatorCurrentLimit;
        talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        mLeftClimberMotor.getConfigurator().apply(talonConfig);
        mRightClimberMotor.getConfigurator().apply(talonConfig);

        mLeftClimberMotor.setPosition(0);
        mRightClimberMotor.setPosition(0);

        mRightBrakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.ClimberConstants.rightClimberBrakeID);
        mLeftBrakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.ClimberConstants.leftClimberBrakeID);

        setDefaultCommand(new RunCommand(() -> {
            setLeftClimbPower(0);
            setRightClimbPower(0);
        },
                this));
    }

    public void setLeftClimbPower(double power) {
        boolean solenoidVal = (power == 0);
        mLeftBrakeSolenoid.set(solenoidVal);
        SmartDashboard.putBoolean("Climber/Left/Brake Solenoid", solenoidVal);
        SmartDashboard.putNumber("Climber/Left/Power", power);
        mLeftClimberMotor.set(power);
    }

    public void setRightClimbPower(double power) {
        boolean solenoidVal = (power == 0);
        mRightBrakeSolenoid.set(solenoidVal);
        SmartDashboard.putBoolean("Climber/Right/Brake Solenoid", solenoidVal);
        SmartDashboard.putNumber("Climber/Right/Power", power);
        mRightClimberMotor.set(power);
    }

    public double getLeftPosition() {
        return mLeftPositionRotations;
    }

    public double getRightPosition() {
        return mRightPositionRotations;
    }

    public boolean isLeftLimitSwitchHit() {
        return mLeftLimitStatus;
    }

    public boolean isRightLimitSwitchHit() {
        return mRightLimitStatus;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        mLeftPositionRotations = mLeftClimberMotor.getPosition().getValueAsDouble();
        mRightPositionRotations = mRightClimberMotor.getPosition().getValueAsDouble();

        mLeftLimitStatus = (mLeftClimberMotor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround);
        mRightLimitStatus = (mLeftClimberMotor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround);

        SmartDashboard.putNumber("Climber/Left/Motor Position", getLeftPosition());
        SmartDashboard.putBoolean("Climber/Left/Limit Switch", isLeftLimitSwitchHit());

        SmartDashboard.putNumber("Climber/Right/Motor Position", getRightPosition());
        SmartDashboard.putBoolean("Climber/Right/Limit Switch", isRightLimitSwitchHit());
    }

    public void setCurrentLimits(double kCurrentLimit) {
        mLeftClimberMotor.getConfigurator().apply(talonConfig.CurrentLimits.withStatorCurrentLimit(kCurrentLimit));
        mRightClimberMotor.getConfigurator().apply(talonConfig.CurrentLimits.withStatorCurrentLimit(kCurrentLimit));
    }
}