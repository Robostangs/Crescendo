package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeMultiple extends Command {
    Intake intake;

    public IntakeMultiple() {
        intake = Intake.getInstance();

        this.addRequirements(intake);
        this.setName("Intake Multiple");
    }

    @Override
    public void initialize() {
        intake.setExtend(true);
        intake.setIntake(1);
        intake.setBelt(1);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setExtend(false);
        intake.setIntake(0);
        intake.setBelt(0);
    }
}
