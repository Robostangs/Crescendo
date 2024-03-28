package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ReturnHome extends Command {
    Arm arm;

    public ReturnHome() {
        arm = Arm.getInstance();

        this.addRequirements(arm);
        this.setName("Return Home");
    }

    @Override
    public void initialize() {
        arm.setMotionMagic(Constants.ArmConstants.SetPoints.kIntake);
        arm.postStatus("Returning Home");
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
