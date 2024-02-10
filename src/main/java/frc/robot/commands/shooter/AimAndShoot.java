package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;

public class AimAndShoot extends Command {
    private Arm mArm;
    private Shooter mShooter;
    private double armSetpoint;
    private double error = 0;
    private Timer timer;
    private boolean debugMode = false;
    // private Supplier<Boolean> feedUntil;

    /**
     * Set the shooter to a specific position and shoots when within 1 degree
     * 
     * @param target in degrees of THE SHOOTER, not the extension bar
     */
    public AimAndShoot(double target) {
        timer = new Timer();
        mArm = Arm.getInstance();
        mShooter = Shooter.getInstance();
        armSetpoint = target;
        // this.feedUntil = null;

        this.setName("Setpoint: " + armSetpoint + " degrees");
        this.addRequirements(mArm, mShooter);
    }

    @Override
    public void initialize() {
        timer.restart();

        SmartDashboard.putString("Shooter/Status", "Charging Up");

        error = armSetpoint - mArm.getArmPosition();
        SmartDashboard.putNumber("Arm/Position Error", error);
        SmartDashboard.putNumber("Arm/Setpoint", armSetpoint);

        if (debugMode) {
            System.out.println("\n*************************** Debug Stats (initialize) ***************************");
            System.out.println("Shooter position: " + mArm.getArmPosition());
            System.out.println("Shooter target position: " + armSetpoint);
            System.out.println("Error: " + error);
            System.out.println("*************************** Debug Stats (initialize) ***************************\n");
        }
        mArm.setArmTarget(armSetpoint);
        mArm.setMotionMagic(armSetpoint);
    }

    @Override
    public void execute() {
        if (!mArm.validSetpoint(armSetpoint)) {
            this.end(true);
        }

        error = armSetpoint - mArm.getArmPosition();

        if (error < 1 && timer.get() > Constants.ShooterConstants.shooterChargeUpTime && mArm.getVelocity() < 5) {
            mShooter.shoot(1, 1);
            SmartDashboard.putString("Shooter/Status", "Shooting");
        } else {
            mShooter.shoot(Constants.ShooterConstants.kFeederFeedForward, 1);
            SmartDashboard.putString("Shooter/Status", "Charging Up");
        }

        if (debugMode) {
            System.out.println("\n*************************** Debug Stats (execute) ***************************");
            System.out.println("Shooter position: " + mArm.getArmPosition());
            System.out.println("Shooter target position: " + armSetpoint);
            System.out.println("Error: " + error);
            System.out.println("*************************** Debug Stats (execute) ***************************\n");
        }
    }

    @Override
    public boolean isFinished() {
        /* Dont do this unless absolutely necessary (do it once the motionMagic works properly */
        return false;
        // if (Robot.isSimulation()) {
        //     return timer.get() > 1;
        // } else {
        //     return mArm.isInRangeOfTarget(armSetpoint);
        // }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            SmartDashboard.putNumber("Constants2.", 0);
        }
        SmartDashboard.putString("Shooter/Status", "Idle");
        mShooter.setBrake(true);
        mShooter.stop();
    }
}