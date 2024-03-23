package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class CancelShooter extends Command {
    Shooter shooter;

    public CancelShooter() {
        shooter = Shooter.getInstance();

        this.addRequirements(shooter);
        this.setName("Cancel Shooter");
    }

    @Override
    public void initialize() {
        shooter.shoot(0, 0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}