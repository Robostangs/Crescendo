package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class Prepare extends Command {
    private Shooter shooter;
    private double Mult;
    public Prepare() {
        shooter = Shooter.getInstance();

        Mult = 1;
        
        this.addRequirements(shooter);
        this.setName("Prepare");
    }
    public Prepare(double Mult) {
        shooter = Shooter.getInstance();
        this.Mult = Mult;

        this.addRequirements(shooter);
        this.setName("Prepare");
    }


    @Override
    public void initialize() {

        shooter.setShooterMotors(1*Mult);
        shooter.postStatus("Preparing");
    }

    @Override
    public boolean isFinished() {
        return false;
        // return shooter.readyToShoot();
    }
}
