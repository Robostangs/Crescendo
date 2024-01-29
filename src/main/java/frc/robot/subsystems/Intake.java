package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    
    public static Intake mIntake;

    public static Intake getInstance() {
        if (mIntake == null)
            mIntake = new Intake();
        return mIntake;
    }

    @Override
    public void periodic() {
        
    }
}
