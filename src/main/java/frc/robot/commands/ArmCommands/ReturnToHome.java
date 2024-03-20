package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;

public class ReturnToHome extends Command {
    Shooter shooter;
    Arm arm;

    public ReturnToHome() {
        shooter = Shooter.getInstance();
        arm = Arm.getInstance();

        this.addRequirements(shooter, arm);
        this.setName("Return To Home");
    }

    @Override
    public void initialize() {
        shooter.shoot(0, 0);
        arm.setMotionMagic(Constants.ArmConstants.SetPoints.kIntake);
    }

    @Override
    public boolean isFinished() {
        return arm.isInRangeOfTarget(Constants.ArmConstants.SetPoints.kIntake);
    }
}
