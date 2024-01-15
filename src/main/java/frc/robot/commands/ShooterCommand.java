package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterCommand extends Command {
    private Shooter mShooter;

    private BooleanSupplier leftPressed, rightPressed;
    private double shootSpeed, feedSpeed;

    public ShooterCommand(Shooter mShooter, BooleanSupplier leftPressed, BooleanSupplier rightPressed) {
        this.mShooter = mShooter;
        addRequirements(mShooter);

        this.leftPressed = leftPressed;
        this.rightPressed = rightPressed;
    }

    @Override
    public void execute() {

        feedSpeed = leftPressed.getAsBoolean() ? 0.3 : 0;
        shootSpeed = rightPressed.getAsBoolean() ? 0.3 : 0;
        
        mShooter.setSpeed(shootSpeed, feedSpeed);
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
