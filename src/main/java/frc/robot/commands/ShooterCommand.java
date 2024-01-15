package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterTesting extends Command {
    public Shooter mShooter;

    public ShooterTesting(Shooter mShooter) {
        this.mShooter = mShooter;
        addRequirements(mShooter);
    }

    @Override
    public void execute() {
        mShooter.setSpeed(1, 1);
    }

    @Override
    public void end(boolean interuppted){
        mShooter.setSpeed(0, 0);
    }
    
    @Override
    public boolean isFinished(){
        return false;
    }
}
