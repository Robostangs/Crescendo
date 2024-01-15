package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.shooterConstants;

public class Shooter extends SubsystemBase {
    
    public static Shooter mShooter;

    private final TalonFX leftMotor = new TalonFX(shooterConstants.leftMotorID, "*");
    private final TalonFX rightMotor = new TalonFX(shooterConstants.rightMotorID, "*");
    private final TalonFX holdingMotor = new TalonFX(shooterConstants.holdingMotorID, "*");

    public static Shooter getInstance() {
        if (mShooter == null)
            mShooter = new Shooter();
        return mShooter;
    }

    public Shooter() {
        leftMotor.setInverted(false);
        rightMotor.setInverted(true);
        holdingMotor.setInverted(false);
    }

    public void setSpeed(double shootingSpeed, double holdingSpeed) {
        leftMotor.set(shootingSpeed);
        rightMotor.set(shootingSpeed);
        holdingMotor.set(holdingSpeed);
    } 
}
