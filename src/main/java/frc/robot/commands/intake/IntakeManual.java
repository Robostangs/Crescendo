package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class IntakeManual extends Command {
    private Intake mIntake = Intake.getInstance();

    public IntakeManual() {
        addRequirements(mIntake);
    }

    @Override
    public void initialize() {
        mIntake.setExtend(true);
    }

    @Override
    public void execute() {
        mIntake.setSpin(IntakeConstants.INTAKE_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        mIntake.setSpin(0);
        mIntake.setExtend(false);
    }
}
