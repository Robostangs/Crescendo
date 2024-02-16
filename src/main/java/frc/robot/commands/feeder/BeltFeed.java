package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class BeltFeed extends Command {
    private Intake mIntake;
    private Shooter mShooter;

    public BeltFeed() {
        mIntake = Intake.getInstance();
        mShooter = Shooter.getInstance();

        this.setName("Belt Feed");
        this.addRequirements(mIntake, mShooter);
    }

    @Override
    public void execute() {
        if (!mIntake.getShooterSensor()) {
            mIntake.setBelt(0.2);
            mShooter.shoot(0.1, 0);
        } else {
            mIntake.setBelt(Constants.IntakeConstants.beltFeedForward);
            mShooter.shoot(0, 0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        mIntake.setBelt(0);
    }
}
