package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Feed extends Command {
    Shooter shooter;

    public Feed() {
        shooter = Shooter.getInstance();

        this.addRequirements(shooter);
        this.setName("Feed");
    }

    @Override
    public void initialize() {
        shooter.postStatus("Feeding");
        shooter.setFeederMotor(Constants.ShooterConstants.feederFeedForward);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setFeederMotor(0);
    }

    @Override
    public boolean isFinished() {
        return Intake.getInstance().getShooterSensor();
    }
}
