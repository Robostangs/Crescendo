package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class FeedAndShoot extends Command {
    private final Shooter mShooter;
    private Timer timer;

    /**
     * This command will activate the feed motor for
     */
    public FeedAndShoot() {
        timer = new Timer();
        mShooter = Shooter.getInstance();
        this.setName("Feed And Shoot");
        this.addRequirements(mShooter);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Shooter/Shooter State", "Charging Up");
        timer.restart();
        // Arm.getInstance().setBrake(true);
        mShooter.setHolding(true);
    }

    @Override
    public void execute() {
        if (timer.get() < Constants.ShooterConstants.shooterChargeUpTime) {
            mShooter.shoot(Constants.ShooterConstants.kFeederFeedForward, 1);
            SmartDashboard.putString("Shooter/Shooter State", "Charging Up");
        } else {
            // mShooter.shoot(1, 1);
            mShooter.shoot(1, 0.95, 1);
            SmartDashboard.putString("Shooter/Shooter State", "Shooting");
        }
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("Shooter/Shooter State", "Idle");

        mShooter.setBrakeMode(true);
        // Arm.getInstance().setBrake(false);
        mShooter.stop();

        // if (!(timer.get() < Constants.ShooterConstants.shooterChargeUpTime)) {
        //     mShooter.setHolding(false);
        // }
    }

    @Override
    public boolean isFinished() {
        return !mShooter.getHolding();
        // return false;
        // return timer.get() > Constants.BeltConstants.shooterChargeUpTime + 0.1;
    }
}
