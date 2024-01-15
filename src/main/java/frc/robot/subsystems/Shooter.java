package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    
    public static Shooter mShooter;

    private final TalonFX leftMotor = new TalonFX(61, "*");
    private final TalonFX rightMotor = new TalonFX(60, "*");

    public static Shooter getInstance() {
        if (mShooter == null)
            mShooter = new Shooter();
        return mShooter;
    }

    public Shooter() {
        leftMotor.setInverted(false);
        rightMotor.setInverted(true);
    }

    public void setSpeed(double leftSpeed, double rightSpeed) {
        leftMotor.set(leftSpeed);
        rightMotor.set(rightSpeed);
    } 
    
}
