package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import java.util.function.BooleanSupplier;

public class FeedAndShoot extends Command {
    private final Shooter mShooter;
    private final Intake mIntake;
    private Timer timer;
    private BooleanSupplier feedUntil;

    /**
     * This command will wait until a piece is in the shooter and then shoot, but if
     * there was already a piece in the shooter then it will wait until the shooter
     * is charged up and then shoot
     */
    public FeedAndShoot() {
        this(null);
    }

    /**
     * This command will wait until a piece is in the shooter and user speciified
     * condition is met, then shoot
     * 
     * @param feedUntil a supplier that returns true when the shooter should shoot,
     *                  false means to charge up
     */
    public FeedAndShoot(BooleanSupplier feedUntil) {
        mShooter = Shooter.getInstance();
        mIntake = Intake.getInstance();
        this.setName("Feed And Shoot");
        this.addRequirements(mShooter, mIntake);
        this.feedUntil = feedUntil;
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Shooter/Status", "Charging Up");
    }

    @Override
    public void execute() {
        if (feedUntil == null) {
            if (mIntake.getShooterSensor()) {
                if (timer == null) {
                    timer = new Timer();
                    timer.restart();
                }

                if (timer.get() < Constants.ShooterConstants.feederChargeUpTime) {
                    mShooter.shoot(Constants.ShooterConstants.feederReverseFeed, 0);
                    SmartDashboard.putString("Shooter/Status", "Reversing Feed");
                } else {
                    mShooter.shoot(0, 1);
                }

                if (mShooter.readyToShoot()) {
                    mShooter.shoot(0.5, 1);
                    SmartDashboard.putString("Shooter/Status", "Shooting");
                    mIntake.setHolding(true);
                }
            } else {
                mShooter.shoot(0.1, 0);
                mIntake.setBelt(Constants.IntakeConstants.beltIntakeSpeed);
            }
        }
        
        else {
            if (mIntake.getShooterSensor()) {
                if (timer == null) {
                    timer = new Timer();
                    timer.restart();
                }

                if (timer.get() < Constants.ShooterConstants.feederChargeUpTime) {
                    mShooter.shoot(Constants.ShooterConstants.feederReverseFeed, 0);
                    SmartDashboard.putString("Shooter/Status", "Reversing Feed");
                } else {
                    mShooter.shoot(0, 1);
                }

                if (feedUntil.getAsBoolean()) {
                    mShooter.shoot(0.5, 1);
                    SmartDashboard.putString("Shooter/Status", "Shooting");
                    mIntake.setHolding(true);
                }

                mIntake.setBelt(0);
            } else {
                mShooter.shoot(0.1, 1);
                mIntake.setBelt(Constants.IntakeConstants.beltIntakeSpeed);
                SmartDashboard.putString("Shooter/Status", "Charging Up");
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("Shooter/Status", "Idle");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
