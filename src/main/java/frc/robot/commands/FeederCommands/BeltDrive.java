package frc.robot.commands.FeederCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class BeltDrive extends Command {
    Intake intake;
    Supplier<Double> manualAdjust;

    public BeltDrive(Supplier<Double> manualAdjust) {
        intake = Intake.getInstance();

        this.manualAdjust = manualAdjust;

        this.addRequirements(intake);
        this.setName("Belt Drive");
    }

    @Override
    public void execute() {
        intake.setBelt(manualAdjust.get());
    }
}