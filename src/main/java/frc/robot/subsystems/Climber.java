// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

    private TalonFX mLeftClimberMotor, mRightClimberMotor;
    private StatusSignal<Double> mLeftPosition, mRightPosition;

    private TalonFXConfiguration talonConfig = new TalonFXConfiguration();

    public Runnable stopClimber = () -> {
        setLeftClimbPower(0);
        setRightClimbPower(0);
    };

    private Climber() {
        mLeftClimberMotor = new TalonFX(Constants.ClimberConstants.LeftMotor.kId, "*");
        mRightClimberMotor = new TalonFX(Constants.ClimberConstants.RightMotor.kId, "*");

        mLeftClimberMotor.setInverted(Constants.ClimberConstants.LeftMotor.kInverted);
        mRightClimberMotor.setInverted(Constants.ClimberConstants.RightMotor.kInverted);

        talonConfig.Feedback.SensorToMechanismRatio = Constants.ClimberConstants.LeftMotor.kGearboxRotationsToMechanismMeters;
        talonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        talonConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.ClimberConstants.LeftMotor.kMaxExtensionMeters;
        talonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        talonConfig.CurrentLimits.StatorCurrentLimit = Constants.ClimberConstants.kDefaultStatorCurrentLimit;
        talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        mLeftClimberMotor.getConfigurator().apply(talonConfig);
        mRightClimberMotor.getConfigurator().apply(talonConfig);

        mLeftClimberMotor.setPosition(0);
        mRightClimberMotor.setPosition(0);

        mLeftPosition = mLeftClimberMotor.getPosition();
        mRightPosition = mRightClimberMotor.getPosition();
    }

    public void setLeftClimbPower(double power) {
        SmartDashboard.putNumber("Climber/Left Power", power);
        mLeftClimberMotor.set(power);
    }

    public void setRightClimbPower(double power) {
        SmartDashboard.putNumber("Climber/Right Power", power);
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

    @Override
    public void periodic() {
        mLeftPosition.refresh();
        mRightPosition.refresh();

        SmartDashboard.putNumber("Climber/Left Motor Position", getLeftPosition());
        SmartDashboard.putNumber("Climber/Right Motor Position", getRightPosition());
    }

    public void setCurrentLimits(double kCurrentLimit) {
        mLeftClimberMotor.getConfigurator().apply(talonConfig.CurrentLimits.withStatorCurrentLimit(kCurrentLimit));
        mRightClimberMotor.getConfigurator().apply(talonConfig);
    }

    private static Climber instance;

    public static Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }

        return instance;
    }

    public void setReverseSoftLimitState(boolean softLimitEnable) {
        mLeftClimberMotor.getConfigurator()
                .apply(talonConfig.SoftwareLimitSwitch.withReverseSoftLimitEnable(softLimitEnable));
        mRightClimberMotor.getConfigurator()
                .apply(talonConfig);
    }

    public void FrickItWeBall(boolean limitOverride) {
        mLeftClimberMotor.getConfigurator().apply(talonConfig.SoftwareLimitSwitch
                .withReverseSoftLimitEnable(!limitOverride).withForwardSoftLimitEnable(!limitOverride));
        mRightClimberMotor.getConfigurator().apply(talonConfig.SoftwareLimitSwitch
                .withReverseSoftLimitEnable(!limitOverride).withForwardSoftLimitEnable(!limitOverride));
    }

    
}