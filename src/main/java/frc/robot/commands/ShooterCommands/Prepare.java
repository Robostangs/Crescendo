package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class Prepare extends Command {
    private Shooter shooter;

    public Prepare() {
        shooter = Shooter.getInstance();

        this.addRequirements(shooter);
        this.setName("Prepare");
    }

    @Override
    public void initialize() {
        shooter.setShooterMotors(1);
        shooter.postStatus("Preparing");
    }

    @Override
    public boolean isFinished() {
        return shooter.readyToShoot();
    }
}
