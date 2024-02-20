package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class BeltFeed extends Command {
    private Intake mIntake;
    private Shooter mShooter;

    public BeltFeed() {
        mIntake = Intake.getInstance();
        mShooter = Shooter.getInstance();

        this.setName("Belt Feed");
        this.addRequirements(mIntake, mShooter);
    }

    @Override
    public void execute() {
        if (!mIntake.getShooterSensor() && mIntake.getHolding()) {
            mIntake.setBelt(Constants.IntakeConstants.beltIntakeSpeed);
            mShooter.shoot(0.1, 0);
        } else {
            if (mIntake.getShooterSensor()) {
                SmartDashboard.putString("Arm/Status", "Waiting To Shoot");
                mShooter.shoot(Constants.ShooterConstants.feederFeedForward, 0);
            } else {
                SmartDashboard.putString("Arm/Status", "Waiting For Note");
                mShooter.shoot(0, 0);

            }
            
            mIntake.setBelt(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        mIntake.setBelt(0);
    }
}
