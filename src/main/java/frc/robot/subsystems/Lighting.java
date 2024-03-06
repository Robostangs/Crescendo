package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Lights;
import frc.robot.Vision.LimelightHelpers;
import frc.robot.subsystems.Drivetrain.Drivetrain;

// TODO
/*
* has a piece -> solid orange
* shooter @ position -> blinking green
* shooter ready to shoot -> solid green
* empty/idle -> cool pattern
*/

public class Lighting extends SubsystemBase {
    private static Lighting mLighting;
    public static Shooter mShooter = Shooter.getInstance();
    public static Intake mIntake =  Intake.getInstance();
    
    private static Spark blinkin;

    public static double lastLight;

    public static double PWMVal;
    public static Timer timer;

    public static Lighting getInstance() {
        if (mLighting == null) {
            mLighting = new Lighting();
        }
        return mLighting;
    }

    public Lighting() {
        blinkin = new Spark(Lights.blinkinPWM_ID);
        timer = new Timer();
    }

    public void setLights(double PWMVal) {
        blinkin.set(PWMVal);
    }

    public void lightsOff() {
        blinkin.set(Lights.kBlack);
    }

    public static double getPWM() {
        return blinkin.get();
    }
    @Override
    public void periodic() {
        if (mIntake.getShooterSensor() && DriverStation.isEnabled()) {
            // mIntake.setHolding(true);

            // LEDs will blink when the arm is at the right setpoint to score
            if (Arm.getInstance().isInRangeOfTarget(Arm.getInstance().calculateArmSetpoint())
                    && Drivetrain.getInstance().isInRangeOfTarget()) {
                if (mShooter.readyToShoot()){
                    blinkin.set(Constants.Lights.kGreen);
                }else{
                    blinkin.set(Constants.Lights.kBlink);
                }
            }

            // LEDs will be on when the arm is not at the right setpoint to score, but the shooter is occupied
            else {
                blinkin.set(Constants.Lights.kOrange);
            }
        }

        // LEDs will be off when the shooter is not occupied or robot is off
        else {
            blinkin.set(Constants.Lights.kLavaPalette);
        }
    }
}