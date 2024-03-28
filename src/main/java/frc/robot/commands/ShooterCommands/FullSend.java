package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class FullSend extends Command {
    Shooter shooter;

    public FullSend() {
        shooter = Shooter.getInstance();

        this.addRequirements(shooter);
        this.setName("Full Send");
    }

    @Override
    public void initialize() {
        shooter.postStatus("SEND IT");
        shooter.shoot(1, 1);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.shoot(0, 0);
    }
}
