package frc.robot.commands.shooter;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
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
        mArm.aimRaw(manualAdjust.get() * Constants.ArmConstants.rateOfMotion);
    }

    @Override
    public void end(boolean interrupted) {
        //TODO change this later???
        // mArm.setMotionMagic(mArm.getArmPosition());
    }
}