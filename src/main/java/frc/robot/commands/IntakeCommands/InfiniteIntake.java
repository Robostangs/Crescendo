
package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class InfiniteIntake extends Command {
    Intake intake;
    double power;

    public InfiniteIntake(double power) {
        this.power = power;
        intake = Intake.getInstance();

        this.addRequirements(intake);
        this.setName("Auto Intaking");
    }

    @Override
    public void initialize() {
        intake.setExtend(true);
        intake.setIntake(power);
        
        intake.setBelt(power);
        // intake.setHolding(true);

        intake.postStatus("Deploying Intake");
        intake.postStatus("Infinitely Intaking");
    }

    @Override
    public void end(boolean interrupted) {
        intake.setExtend(false);
        intake.setIntake(0);
        intake.setBelt(0);
    }
}
