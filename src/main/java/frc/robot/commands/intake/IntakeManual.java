package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeManual extends Command {
    private Intake mIntake = Intake.getInstance();

    public IntakeManual() {
        addRequirements(mIntake);
    }

    @Override
    public void initialize() {
        mIntake.setExtend(true);
        /* Matthew's command will work weird because motors start as soon as command begins, rewrite Matt's code */
    }

    @Override
    public void execute() {
        mIntake.setMotor(1);
    }

    @Override
    public void end(boolean interrupted) {
        mIntake.setMotor(0);
        mIntake.setExtend(false);
    }
}
