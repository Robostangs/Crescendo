package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Lights;
import frc.robot.subsystems.Drivetrain.Drivetrain;

public class Lighting extends SubsystemBase {
    public Shooter mShooter;
    public Intake mIntake;
    
    private Spark blinkin;
    
    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            setLights(Lights.kOrange);
        } 
        
        else if (mIntake.getShooterSensor()) {
            if (mShooter.readyToShootAdvanced() && Drivetrain.getInstance().readyToShoot()) {
                setLights(Lights.kGreen);
            }

            else {
                setLights(Lights.kBlue);
            }
        }

        else {
            setLights(Lights.kRed);
        }
    }

    private Lighting() {
        mShooter = Shooter.getInstance();
        mIntake = Intake.getInstance();

        blinkin = new Spark(Lights.blinkinPWM_ID);
    }

    public void setLights(double PWMVal) {
        blinkin.set(PWMVal);
    }

    public void lightsOff() {
        blinkin.set(Lights.kOff);
    }

    public double getPWM() {
        return blinkin.get();
    }

    private static Lighting mLighting;
    
    public static Lighting getInstance() {
        if (mLighting == null) {
            mLighting = new Lighting();
        }

        return mLighting;
    }

}