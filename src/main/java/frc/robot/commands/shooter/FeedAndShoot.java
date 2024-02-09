package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Shooter;
import java.util.function.BooleanSupplier;

public class FeedAndShoot extends Command {
    private final Shooter mShooter;
    private Timer timer;
    private BooleanSupplier feedUntil;

    /**
     * This command will activate the feed motor for
     */
    public FeedAndShoot() {
        this(() -> true);
    }

    public FeedAndShoot(BooleanSupplier feedUntil) {
        timer = new Timer();
        mShooter = Shooter.getInstance();
        this.setName("Feed And Shoot");
        this.addRequirements(mShooter);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Shooter/Status", "Charging Up");
        timer.restart();
        // Arm.getInstance().setBrake(true);
    }

    @Override
    public void execute() {
        if (timer.get() < Constants.ShooterConstants.shooterChargeUpTime && feedUntil.getAsBoolean()) {
            mShooter.shoot(Constants.ShooterConstants.kFeederFeedForward, 1);
            SmartDashboard.putString("Shooter/Status", "Charging Up");
        } else {
            mShooter.shoot(1, 1);
            // mShooter.shoot(1, 0.95, 1);
            SmartDashboard.putString("Shooter/Status", "Shooting");
        }
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("Shooter/Status", "Idle");

        mShooter.setBrakeMode(true);
        // Arm.getInstance().setBrake(false);
        mShooter.stop();

        // if (!(timer.get() < Constants.ShooterConstants.shooterChargeUpTime)) {
        //     mShooter.setHolding(false);
        // }
    }

    @Override
    public boolean isFinished() {
        if (Robot.isReal()) {
            return !mShooter.getHolding();
        }
        return timer.get() < 0.5;
    }
}
