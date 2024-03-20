package frc.robot.commands.FeederCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class PassToShooter extends Command {
    private Intake intake;
    private Shooter shooter;

    public PassToShooter() {
        intake = Intake.getInstance();
        shooter = Shooter.getInstance();
        this.setName("Pass to Shooter");
        this.addRequirements(intake, shooter);
    }

    @Override
    public void execute() {
        intake.setBelt(Constants.IntakeConstants.beltIntakeSpeed);
        shooter.shoot(0, Constants.ShooterConstants.feederFeedForward);
    }

    @Override
    public boolean isFinished() {
        return intake.getShooterSensor();
    }
}
