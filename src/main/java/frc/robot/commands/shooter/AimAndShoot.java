package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AimAndShoot extends Command {
    private Arm mArm;
    private Shooter mShooter;
    private Intake mIntake;

    private double armSetpoint;
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
        mIntake = Intake.getInstance();
        armSetpoint = target;

        this.setName("Aim and Shoot: " + armSetpoint + " degrees");
        this.addRequirements(mArm, mShooter);
    }

    public AimAndShoot() {
        this(Arm.getInstance().calculateArmSetpoint());
    }

    @Override
    public void initialize() {
        if (!mArm.validSetpoint(armSetpoint)) {
            this.end(true);
        }

        SmartDashboard.putString("Shooter/Status", "Charging Up");
        
        SmartDashboard.putNumber("Arm/Position Error", (armSetpoint - mArm.getArmPosition()));
        SmartDashboard.putNumber("Arm/Setpoint", armSetpoint);
        
        if (debugMode) {
            System.out.println("\n*************************** Debug Stats (initialize) ***************************");
            System.out.println("Shooter position: " + mArm.getArmPosition());
            System.out.println("Shooter target position: " + armSetpoint);
            System.out.println("Error: " + (armSetpoint - mArm.getArmPosition()));
            System.out.println("*************************** Debug Stats (initialize) ***************************\n");
        }
        
        timer.restart();
        mArm.setArmTarget(armSetpoint);
    }

    @Override
    public void execute() {
        if (mArm.isInRangeOfTarget(armSetpoint, 1) && timer.get() > Constants.ShooterConstants.shooterChargeUpTime && mArm.getVelocity() < 5) {
            mShooter.shoot(0.1, 1);
            SmartDashboard.putString("Shooter/Status", "Shooting");
        } else if (mIntake.getShooterSensor() || Robot.isSimulation()) {
            mArm.setMotionMagic(armSetpoint);
            mShooter.shoot(Constants.ShooterConstants.kFeederFeedForward, 1);
            SmartDashboard.putString("Shooter/Status", "Charging Up");
        } else {
            mShooter.shoot(0.1, 1);
            SmartDashboard.putString("Shooter/Status", "Waiting for note");
        }

        if (debugMode) {
            System.out.println("\n*************************** Debug Stats (execute) ***************************");
            System.out.println("Shooter position: " + mArm.getArmPosition());
            System.out.println("Shooter target position: " + armSetpoint);
            System.out.println("Error: " + (armSetpoint - mArm.getArmPosition()));
            System.out.println("*************************** Debug Stats (execute) ***************************\n");
        }
    }

    @Override
    public boolean isFinished() {
        return false;
        // return (mArm.isInRangeOfTarget(armSetpoint, 1) && timer.get() > Constants.ShooterConstants.shooterChargeUpTime && mArm.getVelocity() < 5 && !mIntake.getShooterSensor());

        /* Dont do this unless absolutely necessary (do it once the motionMagic works properly) */
        // if (Robot.isSimulation()) {
        //     return timer.get() > 1;
        // } else {
        //     return mArm.isInRangeOfTarget(armSetpoint);
        // }
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("Shooter/Status", "Idle");
        mShooter.stop();
    }
}