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
    // private Timer setPoinTimer;
    private boolean debugMode = false;
    private boolean autoAim = false;

    private boolean isFinishedBool = false;
    private boolean atSetpoint = false;

    /**
     * Set the shooter to a specific position and shoots when within 1 degree
     * 
     * @param target in degrees of THE SHOOTER, not the extension bar
     */
    public AimAndShoot(double target) {
        mArm = Arm.getInstance();
        mShooter = Shooter.getInstance();
        mIntake = Intake.getInstance();
        armSetpoint = target;
        isFinishedBool = false;
        autoAim = false;

        this.setName("Aim and Shoot: " + armSetpoint + " degrees");
        this.addRequirements(mArm, mShooter, mIntake);
    }

    /**
     * Calculates the desired setpoint of the arm using robotPose and then charges
     * the shooter motors so that when the arm gets to the correct position, it is
     * ready to feed and shoot
     */
    public AimAndShoot() {
        mArm = Arm.getInstance();
        mShooter = Shooter.getInstance();
        mIntake = Intake.getInstance();
        isFinishedBool = false;
        autoAim = true;
        atSetpoint = false;

        this.setName("Auto aim and shoot");
        this.addRequirements(mArm, mShooter, mIntake);
    }

    @Override
    public void initialize() {
        isFinishedBool = false;

        if (autoAim) {
            armSetpoint = mArm.calculateArmSetpoint();
        }

        if (!mArm.validSetpoint(armSetpoint)) {
            this.end(true);
        }

        SmartDashboard.putString("Shooter/Status", "Charging Up");

        if (debugMode) {
            System.out.println("\n*************************** Debug Stats (initialize) ***************************");
            System.out.println("Shooter position: " + mArm.getArmPosition());
            System.out.println("Shooter target position: " + armSetpoint);
            System.out.println("Error: " + (armSetpoint - mArm.getArmPosition()));
            System.out.println("*************************** Debug Stats (initialize) ***************************\n");
        }

        // mArm.setArmTarget(armSetpoint);
    }

    @Override
    public void execute() {
        if (!mIntake.getShooterSensor()) { // If shooter is empty
            mShooter.shoot(Constants.ShooterConstants.feederFeedForward, 1);

            mArm.setMotionMagic(Constants.ArmConstants.SetPoints.kIntake);
            if (!isFinishedBool) {
                SmartDashboard.putString("Shooter/Status", "Waiting for note");
                mIntake.setBelt(0.3);
            } else {
                SmartDashboard.putString("Shooter/Status", "Returning to Idle");
                mIntake.setBelt(0);
            }
        }

        // If the arm is within 1 degree of the target
        else if (mArm.isInRangeOfTarget(armSetpoint, 1) && mArm.getVelocity() < 1) {
            mShooter.shoot(0.5, 1);
            SmartDashboard.putString("Shooter/Status", "Shooting");
            /* TODO: this may not work cuz it cancels the command so quick but try it */
            // if (atSetpointTimer.get() < )
            isFinishedBool = true;
        }

        // If shooter is loaded but arm is not in position
        else {
            SmartDashboard.putString("Shooter/Status", "Traveling to Setpoint");
            if (timer == null) {
                timer = new Timer();
                timer.restart();
            }

            // 0.23sec is the time it takes for the note to travel into the shooter but not
            // hit
            // the shooter wheels
            if (timer.get() < Constants.ShooterConstants.feederChargeUpTime) {
                mShooter.shoot(0.055, 1);
                SmartDashboard.putString("Shooter/Status", "Charging Up");
            } else {
                if (armSetpoint < mArm.getArmPosition()) {
                    mShooter.shoot(Constants.ShooterConstants.feederFeedForward, 1);
                } else {
                    mShooter.shoot(0, 1);
                }
            }

            mArm.setMotionMagic(armSetpoint);
        }

        if (debugMode) {
            System.out.println("\n*************************** Debug Stats (execute) ***************************");
            System.out.println("Shooter position: " + mArm.getArmPosition());
            System.out.println("Shooter target position: " + armSetpoint);
            System.out.println("Error: " + (armSetpoint - mArm.getArmPosition()));
            System.out.println("*************************** Debug Stats (execute) ***************************\n");
        }
        // SmartDashboard.putNumber("Arm/Position Error", (armSetpoint -
        // mArm.getArmPosition()));
    }

    @Override
    public boolean isFinished() {
        // return false;
        if (Robot.isSimulation()) {
            return false;
        }

        return false;

        // return false;
        // return (mArm.isInRangeOfTarget(armSetpoint, 1) &&
        // !mIntake.getShooterSensor());
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("Shooter/Status", "Idle");
        mShooter.stop();
    }
}