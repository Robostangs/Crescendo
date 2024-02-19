package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class QuickFeed extends Command {
    Intake mIntake;
    Shooter mShooter;

    public QuickFeed() {
        mIntake = Intake.getInstance();
        mShooter = Shooter.getInstance();

        this.setName("Quick Feed");
        this.addRequirements();
    }

    @Override
    public void execute() {
        mIntake.setBelt(0.5);
        mShooter.shoot(0.05, 0);
    }
}
