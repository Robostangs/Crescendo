package frc.robot.commands.ArmCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class FineAdjust extends Command {
    private Arm arm;
    private Supplier<Double> manualAdjust;
    
    /**
     * Command to adjust the arm based on a supplied value 
     * 
     * @param manualAdjust how much power to adjust the arm by
     */
    public FineAdjust(Supplier<Double> manualAdjust) {

        arm = Arm.getInstance();
        this.manualAdjust = manualAdjust;

        this.addRequirements(arm);
        this.setName("Fine Adjust");
    }

    @Override
    public void initialize() {
        arm.postStatus("Manually Adjusting Arm");
    }

    @Override
    public void execute() {
        arm.aimRaw(manualAdjust.get() * Constants.ArmConstants.rateOfMotion);
    }

    @Override
    public void end(boolean interrupted) {
        arm.setMotionMagic(arm.getArmPosition());
        arm.postStatus("Holding " + arm.getArmPosition() + " degrees");
    }
}