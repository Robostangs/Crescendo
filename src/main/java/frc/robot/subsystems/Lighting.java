package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drivetrain.Drivetrain;

public class Lighting extends SubsystemBase {
    private DigitalOutput enabled, loaded, readyToShoot;


    @Override
    public void periodic() {
        enabled.set(DriverStation.isEnabled());
        loaded.set(Intake.getInstance().getShooterSensor());
        readyToShoot.set(Intake.getInstance().getShooterSensor() && Shooter.getInstance().readyToShootAdvanced()
                && Drivetrain.getInstance().readyToShoot());

        // if (DriverStation.isDisabled()) {
        // enabled.set(false);
        // setLights(Lights.kOrange);
        // }

        // else if (mIntake.getShooterSensor()) {
        // if (mShooter.readyToShootAdvanced() &&
        // Drivetrain.getInstance().readyToShoot()) {
        // setLights(Lights.kGreen);
        // }

        // else {
        // setLights(Lights.kBlue);
        // }
        // }

        // else {
        // setLights(Lights.kRed);
        // }
    }

    private Lighting() {
        enabled = new DigitalOutput(0);
        loaded = new DigitalOutput(1);
        readyToShoot = new DigitalOutput(2);
    }

    private static Lighting mLighting;

    public static Lighting getInstance() {
        if (mLighting == null) {
            mLighting = new Lighting();
        }

        return mLighting;
    }


}