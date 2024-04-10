package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class CancelShooter extends Command {
    Shooter shooter;
    /**
     * Sets the feeder and shooter wheels to 0
     */
    public static Runnable CancelShooter = () -> {
        Shooter shooter = Shooter.getInstance();
        shooter.shoot(0, 0);
        shooter.postStatus("Stopping Shooter");
    };
    /**
     * A command that sets the feeder and shooter wheels to 0
     */
    public CancelShooter() {
        shooter = Shooter.getInstance();

        this.addRequirements(shooter);
        this.setName("Cancel Shooter");
    }

    @Override
    public void initialize() {
        shooter.shoot(0, 0);
        shooter.postStatus("Stopping Shooter");
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}