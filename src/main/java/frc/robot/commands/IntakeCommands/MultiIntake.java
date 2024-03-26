package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class MultiIntake extends Command {
    Intake intake;

    public MultiIntake() {
        intake = Intake.getInstance();

        this.addRequirements(intake);
        this.setName("Intake Multiple");
    }

    @Override
    public void initialize() {
        intake.setExtend(true);
        intake.setIntake(Constants.IntakeConstants.intakeMotorSpeed);
        intake.setBelt(Constants.IntakeConstants.beltIntakeSpeed);
    }

    @Override
    public void execute() {
        if (intake.getShooterSensor()) {
            intake.setBelt(0.3);
            intake.setIntake(0.3);
        }

        else {
            intake.setBelt(Constants.IntakeConstants.beltIntakeSpeed);
            intake.setIntake(Constants.IntakeConstants.intakeMotorSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.setExtend(false);
        intake.setIntake(0);
        intake.setBelt(0);
    }
}
