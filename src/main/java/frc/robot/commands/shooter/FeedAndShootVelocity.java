package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import java.util.function.BooleanSupplier;

public class FeedAndShootVelocity extends Command {
    private final Shooter mShooter;
    private Timer timer;
    private BooleanSupplier feedUntil;

    /* TODO: this must require the intake subsytem and set the belt motor to 100% */

    /**
     * This command will activate the feed motor for
     */
    public FeedAndShootVelocity() {
        this(null);
    }

    public FeedAndShootVelocity(BooleanSupplier feedUntil) {
        timer = new Timer();
        mShooter = Shooter.getInstance();
        this.setName("Feed And Shoot");
        this.addRequirements(mShooter);
        this.feedUntil = feedUntil;
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Shooter/Status", "Charging Up");
        timer.restart();
    }

    @Override
    public void execute() {
        if (feedUntil == null) {
            if (timer.get() < Constants.ShooterConstants.shooterChargeUpTime) {
                mShooter.shoot(Constants.ShooterConstants.kFeederFeedForward, 1);
                SmartDashboard.putString("Shooter/Status", "Charging Up");
            } else {
                mShooter.shoot(0.1, 1);
                SmartDashboard.putString("Shooter/Status", "Shooting");
            }
        } else {
            if (feedUntil.getAsBoolean()) {
                mShooter.shoot(0.1, 1);
                SmartDashboard.putString("Shooter/Status", "Shooting");
            } else {
                mShooter.shoot(Constants.ShooterConstants.kFeederFeedForward, 1);
                SmartDashboard.putString("Shooter/Status", "Charging Up");
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("Shooter/Status", "Idle");

        mShooter.setBrake(true);
        mShooter.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
