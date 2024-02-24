package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// TODO: this doesnt work with DeployIntake command
public class AutoShoot extends Command {
    private Arm mArm;
    private Shooter mShooter;
    private Intake mIntake;
    private Timer timer;
    private double armSetpoint;
    private boolean isFinished;

    private static final double feedTime = 1;

    public AutoShoot() {
        mArm = Arm.getInstance();
        mShooter = Shooter.getInstance();
        mIntake = Intake.getInstance();

        isFinished = false;
        timer = new Timer();

        this.setName("Auto Shoot");
        this.addRequirements(mArm, mShooter, mIntake);
    }

    @Override
    public void initialize() {
        timer.restart();
        isFinished = false;
    }

    @Override
    public void execute() {
        armSetpoint = mArm.calculateArmSetpoint();

        // feed the piece further into the shooter if there is nothing in the shooter
        // TODO: tune the feedTime and feederSetValue
        if (timer.get() < feedTime && mIntake.getShooterSensor()) {
            mShooter.shoot(0.04, 1);
            mIntake.setBelt(0.5);
        }

        // if at the setpoint and the shooters are charged up and the piece has been fed
        else if (mArm.isInRangeOfTarget(armSetpoint) && mShooter.readyToShoot()) {
            mShooter.shoot(Constants.ShooterConstants.feederShootValue, 1);
            System.out.println("Shooting");
            isFinished = true;
        }

        // if the piece has been pushed further into the shooter but is not yet at the
        // position, or the shooter is not charged up
        else {
            mArm.setMotionMagic(armSetpoint);
            mShooter.shoot(0, 1);
        }

    }

    @Override
    public boolean isFinished() {
        // System.out.println(isFinished);
        if (isFinished) {
            System.out.println("AutoShoot Finished");
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        // get it out of the shooter to avoid penalties
        if (interrupted) {
            mShooter.shoot(1, 1);
        } else {
            System.out.println("AutoShoot Ended without Interruption");
        }

        mArm.setMotionMagic(Constants.ArmConstants.SetPoints.kIntake);
    }
}
