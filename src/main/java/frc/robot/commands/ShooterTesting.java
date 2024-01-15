package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterTesting extends Command {
    public Shooter mShooter;
    public DoubleSupplier leftTrig, rightTrig;
    public double intakeSpeed, shooterSpeed;

    public ShooterTesting(Shooter mShooter, DoubleSupplier leftTrig, DoubleSupplier rightTrig) {
        this.mShooter = mShooter;
        this.leftTrig = leftTrig;
        this.rightTrig = rightTrig;
        addRequirements(mShooter);
    }

    @Override
    public void execute() {

        intakeSpeed = leftTrig.getAsDouble();
        shooterSpeed = rightTrig.getAsDouble();
        if (intakeSpeed > shooterSpeed){
            mShooter.setSpeed(-intakeSpeed, -intakeSpeed);
        }else{
            mShooter.setSpeed(shooterSpeed, shooterSpeed);
        }
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
