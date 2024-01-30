package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    
    public static Shooter mShooter;

    private final TalonFX leftShootMotor = new TalonFX(ShooterConstants.LEFT_SHOOT_MOTOR_ID, "*");
    private final TalonFX rightShootMotor = new TalonFX(ShooterConstants.RIGHT_SHOOT_MOTOR_ID, "*");
    private final TalonFX feedMotor = new TalonFX(ShooterConstants.FEED_MOTOR_ID, "*");
    private final TalonFX wristMotor = new TalonFX(ShooterConstants.WRIST_MOTOR_ID, "*");
    private final CANcoder wristEncoder = new CANcoder(ShooterConstants.WRIST_ENCODER_ID, "*");

    public static Shooter getInstance() {
        if (mShooter == null)
            mShooter = new Shooter();
        return mShooter;
    }

    public Shooter() {
        leftShootMotor.setInverted(ShooterConstants.LEFT_SHOOT_INVERT);
        rightShootMotor.setInverted(ShooterConstants.RIGHT_SHOOT_INVERT);
        feedMotor.setInverted(ShooterConstants.FEED_INVERT);
        wristMotor.setInverted(ShooterConstants.WRIST_INVERT);
    }
    
    public void setShooter(double speed) {
        leftShootMotor.set(speed);
        rightShootMotor.set(speed);
    }

    public void setFeeder(double speed) {
        feedMotor.set(speed);
    } 

    public void setWristVoltage(double voltage) {
        wristMotor.set(voltage);
    }

    public double getWristEncoderVal() {
        return wristEncoder.getAbsolutePosition().getValueAsDouble()-ShooterConstants.WRIST_ENCODER_OFFSET;
    }

    public double getWristSpeed() {
        return wristEncoder.getVelocity().getValueAsDouble();
    }

    public double getAvgRealShootSpeed() {
        return (leftShootMotor.getVelocity().getValueAsDouble() + rightShootMotor.getVelocity().getValueAsDouble())/2;
    }
}
