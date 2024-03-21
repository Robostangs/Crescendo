package frc.robot.commands.ArmCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class FineAdjust extends Command {
    Arm arm = Arm.getInstance();
    Supplier<Double> manualAdjust;

    public FineAdjust(Supplier<Double> manualAdjust) {
        this.manualAdjust = manualAdjust;

        this.addRequirements(arm);
        this.setName("Fine Adjust");
    }

    @Override
    public void execute() {
        arm.aimRaw(manualAdjust.get() * Constants.ArmConstants.rateOfMotion);
    }

    @Override
    public void end(boolean interrupted) {
        arm.setMotionMagic(arm.getArmPosition());
    }
}