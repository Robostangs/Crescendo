package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class MultiIntake extends Command {
    Intake intake;
/**
 * runs belt and then once it's in the shooter it slows belt down instead of stopping
 */
    public MultiIntake() {
        intake = Intake.getInstance();

        this.addRequirements(intake);
        this.setName("Multi Intake");
    }

    @Override
    public void initialize() {
        intake.setExtend(true);
        intake.setIntake(Constants.IntakeConstants.intakeMotorSpeed);
        intake.setBelt(Constants.IntakeConstants.beltIntakeSpeed);
        intake.postStatus("Intaking Multiple");
    }

    @Override
    public void execute() {
        if (intake.getShooterSensor()) {
            intake.setBelt(0.3);
            intake.setIntake(0.6);
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
