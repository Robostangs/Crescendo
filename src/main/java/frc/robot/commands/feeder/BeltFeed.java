package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class BeltFeed extends Command {
    private Intake mIntake;
    private Shooter mShooter;
    public boolean deployIntake = false;

    /**
     * Default command for the belt and the shooter and intake
     */
    public BeltFeed() {
        mIntake = Intake.getInstance();
        mShooter = Shooter.getInstance();

        this.setName("Belt Feed");
        this.addRequirements(mIntake, mShooter);

        deployIntake = false;
    }

    @Override
    public void initialize() {
        if (mShooter.readyToShoot()) {
        }

        // mIntake.setHolding(false);
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
            mShooter.setShoot(Constants.ShooterConstants.feederFeedForward, 0);
        }

        // if there is a piece in the shooter
        else if (mIntake.getShooterSensor()) {
            mIntake.setExtend(false);
            mIntake.setIntake(0);
            mIntake.setBelt(0);

            deployIntake = false;

            mShooter.stop();

            SmartDashboard.putString("Shooter/Status", "Waiting to Shoot");
            SmartDashboard.putString("Intake/Status", "Waiting to Shoot");
            SmartDashboard.putString("Arm/Status", "Waiting to Shoot");
        }

        // if the robot does not currently have a piece in the intake or the shooter
        else {
            SmartDashboard.putString("Shooter/Status", "Waiting For Note");
            SmartDashboard.putString("Intake/Status", "Waiting For Note");
            SmartDashboard.putString("Arm/Status", "Waiting For Note");

            mIntake.setExtend(false);
            mIntake.setIntake(0);
            mIntake.setBelt(0);
            mShooter.stop();
        }

    }

    @Override
    public void end(boolean interrupted) {
        mIntake.setBelt(0);
        mIntake.setIntake(0);
        
        deployIntake = false;
    }
}
