package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class BeltFeed extends Command {
    private Intake mIntake;

    public BeltFeed() {
        mIntake = Intake.getInstance();
        this.setName("Belt Feed");
        this.addRequirements(mIntake);
    }

    @Override
    public void execute() {
        if (!mIntake.getShooterSensor()) {
            mIntake.setBelt(0.2);
        } else {
            mIntake.setBelt(Constants.IntakeConstants.beltFeedForward);
        }
    }

    @Override
    public void end(boolean interrupted) {
        mIntake.setBelt(0);
    }
}
