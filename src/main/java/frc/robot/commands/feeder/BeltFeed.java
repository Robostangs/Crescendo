package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class BeltFeed extends Command {
    private Intake mIntake;
    private Shooter mShooter;
    private Timer timer = null;
    private double FeedForwardTime = 0.5;

    public BeltFeed() {
        mIntake = Intake.getInstance();
        mShooter = Shooter.getInstance();

        this.setName("Belt Feed");
        this.addRequirements(mIntake, mShooter);
    }

    @Override
    public void execute() {
        // if there is nothing in the shooter but the robot has something in the intake
        if (!mIntake.getShooterSensor() && mIntake.getHolding()) {
            mIntake.setBelt(Constants.IntakeConstants.beltIntakeSpeed);
            mShooter.shoot(0.1, 0);
        }

        // if there is a piece in the shooter
        else if (mIntake.getShooterSensor()) {
            if (timer == null) {
                timer = new Timer();
                timer.start();
            }

            // first push the piece all the way in
            if (timer.get() < FeedForwardTime) {
                mShooter.shoot(0.1, 0);
            }
            
            // reversing the feed and pushing the note out of the shooter to charge up the
            // shooter motors
            else if (timer.get() < Constants.ShooterConstants.feederChargeUpTime + FeedForwardTime) {
                mShooter.shoot(Constants.ShooterConstants.feederReverseFeed, 0);
                SmartDashboard.putString("Shooter/Status", "Reversing Feed");
            }

            // piece has been reversed
            else {
                mShooter.stop();
                SmartDashboard.putString("Shooter/Status", "Waiting to Shoot");
            }
            mIntake.setBelt(0);
        }

        // if the robot does not currently have a piece in the intake
        else {
            SmartDashboard.putString("Arm/Status", "Waiting For Note");
            mShooter.shoot(0, 0);
            timer = null;
            mIntake.setBelt(0);
        }

    }

    @Override
    public void end(boolean interrupted) {
        mIntake.setBelt(0);
    }
}
