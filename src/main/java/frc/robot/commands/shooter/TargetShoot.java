package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

public class TargetShoot extends Command{
    private Shooter mShooter;
    private PIDController wristPID = new PIDController(ShooterConstants.wristP, ShooterConstants.wristI, ShooterConstants.wristD);
    private double speakerDistance;
    
    public TargetShoot(Shooter mShooter){
        this.mShooter = mShooter;
        addRequirements(mShooter);
    }

    private double getSpeakerDistance() {
        
    }

    private void aimShooter() {
        
    }
}
