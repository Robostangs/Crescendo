package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class Prepare extends Command {
    private Shooter shooter;

    public Prepare() {
        shooter = Shooter.getInstance();

        this.setName("Prepare");
        this.addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setShooterMotors(1);  
    }

    @Override
    public boolean isFinished() {
        return shooter.readyToShoot();
    }
}
