package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class QuickFeed extends Command {
    Intake mIntake;
    Shooter mShooter;

    public QuickFeed() {
        mIntake = Intake.getInstance();
        mShooter = Shooter.getInstance();

        this.setName("Quick Feed");
        this.addRequirements(mIntake, mShooter);
    }

    @Override
    public void execute() {
        SmartDashboard.putString("Intake/Status", "Quick Feed");
        SmartDashboard.putString("Shooter/Status", "Quick Feed");
        mIntake.setBelt(0.5);
        //TODO reversed this motor
        mShooter.shoot(0.5, 0);
    }
}
