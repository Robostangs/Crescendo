package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterTesting extends Command {
    public Shooter mShooter;
    public DoubleSupplier leftTrig, rightTrig;
    public double leftSpeed, rightSpeed;

    public ShooterTesting(Shooter mShooter, DoubleSupplier leftTrig, DoubleSupplier rightTrig) {
        this.mShooter = mShooter;
        this.leftTrig = leftTrig;
        this.rightTrig = rightTrig;
        addRequirements(mShooter);
    }

    @Override
    public void execute() {
        if (leftTrig.getAsDouble() == 0)
            leftSpeed = -0.1;
        else
            leftSpeed = leftTrig.getAsDouble();

        if (rightTrig.getAsDouble() == 0)
            rightSpeed = -0.1;
        else
            rightSpeed = rightTrig.getAsDouble();
        
        mShooter.setSpeed(leftSpeed, rightSpeed);
    }
}
