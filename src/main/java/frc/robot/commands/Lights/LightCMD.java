package frc.robot.commands.Lights;

import org.opencv.ml.Ml;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.Shooter;;

public class LightCMD extends InstantCommand {
    private final Lighting mLighting = Lighting.getInstance();
    private final Shooter mShooter = Shooter.getInstance();
    Timer timer = Lighting.timer;

  

    public LightCMD() {
        this.addRequirements(mLighting, mShooter);
    }

    @Override
    public void initialize() {
        timer.restart();

    }
    
    @Override
    public void execute() {
        // mLighting.setLights(PWMVal);
        if (mShooter.readyToShootAdvanced()){ // nothing in it & ready to feed & shoot
            mLighting.setLights(readyToShootAndFeed);
        }else if (mShooter.readyToShoot()){ // stuff in it & ready to shoot
            mLighting.setLights(readyToShoot);
        }else{
            mLighting.setLights( Math.sin(timer.get()*cycleMulti)*idleOrange + Math.cos(timer.get()*cycleMulti)*idleBlack); // circle
        }
    }
}