package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    
    public static Shooter mShooter;

    private final TalonFX leftShootMotor = new TalonFX(ShooterConstants.leftShootID, "*");
    private final TalonFX rightShootMotor = new TalonFX(ShooterConstants.rightShootID, "*");
    private final TalonFX feedMotor = new TalonFX(ShooterConstants.feedID, "*");
    private final TalonFX wristMotor = new TalonFX(ShooterConstants.wristID, "*");;

    public static Shooter getInstance() {
        if (mShooter == null)
            mShooter = new Shooter();
        return mShooter;
    }

    public Shooter() {
        leftShootMotor.setInverted(ShooterConstants.leftShootInvert);
        rightShootMotor.setInverted(ShooterConstants.rightShootInvert);
        feedMotor.setInverted(ShooterConstants.feedInvert);
        wristMotor.setInverted(ShooterConstants.wristInvert);
    }
    
    public void setShooter(double speed) {
        leftShootMotor.set(speed);
        rightShootMotor.set(speed);
    }

    public void setFeeder(double speed) {
        feedMotor.set(speed);
    } 

    public void setWrist(double speed) {
        wristMotor.set(speed);
    }
}
