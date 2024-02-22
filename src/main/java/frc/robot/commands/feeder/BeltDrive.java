package frc.robot.commands.feeder;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class BeltDrive extends Command {
    private final Intake mIntake = Intake.getInstance();
    private Supplier<Double> manualAdjust;

    public BeltDrive(Supplier<Double> manualAdjust) {
        this.manualAdjust = manualAdjust;
        this.setName("Belt Drive");
        this.addRequirements(mIntake);
    }

    @Override
    public void execute() {
        mIntake.setBelt(manualAdjust.get());
    }
}