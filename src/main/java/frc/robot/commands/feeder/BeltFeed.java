package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivetrain.Drivetrain;

public class BeltFeed extends Command {
    private Intake mIntake;
    private Shooter mShooter;
    private Timer timer = null;
    
    private double FeedForwardTime = 0.5;
    private double chargeBufferTime = 0.25;
    public boolean deployIntake = false;

    /**
     * Default command for the belt and the shooter
     */
    public BeltFeed() {
        mIntake = Intake.getInstance();
        mShooter = Shooter.getInstance();

        this.setName("Belt Feed");
        this.addRequirements(mIntake, mShooter);

        timer = null;
    }

    @Override
    public void execute() {
        // if there is nothing in the shooter but the robot has something in the intake
        if (!mIntake.getShooterSensor() && mIntake.getHolding()) {
            mIntake.setExtend(deployIntake);
            if (deployIntake) {
                mIntake.setIntake(1);
            } 
            
            else {
                mIntake.setIntake(0);
            }
            mIntake.setBelt(Constants.IntakeConstants.beltIntakeSpeed);
            mShooter.setShoot(Constants.ShooterConstants.feederIntakeValue, Constants.ShooterConstants.shooterReverseSpeed);
        }

        // if there is a piece in the shooter
        else if (mIntake.getShooterSensor()) {
            mIntake.setExtend(false);
            mIntake.setIntake(0);
            mIntake.setBelt(0);

            if (timer == null) {
                timer = new Timer();
                timer.start();
            }

            // first push the piece all the way in
            if (timer.get() < FeedForwardTime) {
                mShooter.setShoot(0.15, Constants.ShooterConstants.shooterReverseSpeed);
            }

            // reversing the feed and pushing the note out of the shooter to charge up the
            // shooter motors
            else if (timer.get() < Constants.ShooterConstants.feederChargeUpTime + FeedForwardTime) {
                mShooter.shoot(Constants.ShooterConstants.feederReverseFeed, 
                        Constants.ShooterConstants.shooterReverseSpeed);
                SmartDashboard.putString("Shooter/Status", "Reversing Feed");
                SmartDashboard.putString("Intake/Status", "Waiting to Shoot");
                SmartDashboard.putString("Arm/Status", "Waiting to Shoot");

            } else if (timer.get() < Constants.ShooterConstants.feederChargeUpTime + FeedForwardTime + chargeBufferTime) {
                mShooter.stop();
            }

            // piece has been reversed
            else {
                if (Drivetrain.getInstance().getDistanceToSpeaker() < 5) {
                    mShooter.shoot(0, 0.6);
                }
                // if the thing above doesnt work then try the thing below

                // mShooter.stop();
                
                SmartDashboard.putString("Shooter/Status", "Waiting to Shoot");
                SmartDashboard.putString("Intake/Status", "Waiting to Shoot");
                SmartDashboard.putString("Arm/Status", "Waiting to Shoot");
            }
        }

        // if the robot does not currently have a piece in the intake or the shooter
        else {
            SmartDashboard.putString("Shooter/Status", "Waiting For Note");
            SmartDashboard.putString("Intake/Status", "Waiting For Note");
            SmartDashboard.putString("Arm/Status", "Waiting For Note");

            mIntake.setExtend(false);
            mIntake.setIntake(0);
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
