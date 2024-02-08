package frc.robot.commands.shooter;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class FineAdjust extends Command {
    private final Arm mArm = Arm.getInstance();
    private Supplier<Double> manualAdjust;


    public FineAdjust(Supplier<Double> manualAdjust) {
        this.manualAdjust = manualAdjust;
        this.setName("Fine Adjust");
        this.addRequirements(mArm);
    }

    @Override
    public void execute() {
        mArm.aim(manualAdjust.get());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
