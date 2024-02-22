package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivetrain.Drivetrain;

public class AimAndShoot extends Command {
    private Arm mArm;
    private Shooter mShooter;
    private Intake mIntake;

    private Timer timer;
    private double armSetpoint;
    private double shootSpeed = 1;

    private boolean autoAim = false;
    private boolean isFinishedBool = false;

    private boolean debugMode = false;

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

        this.setName("Auto aim and shoot");
        this.addRequirements(mArm, mShooter, mIntake);
    }

    @Override
    public void initialize() {
        isFinishedBool = false;

        timer = null;

        if (autoAim) {
            armSetpoint = mArm.calculateArmSetpoint();
        }

        SmartDashboard.putString("Shooter/Status", "Charging Up");

        if (debugMode) {
            System.out.println("\n*************************** Debug Stats (initialize) ***************************");
            System.out.println("Shooter position: " + mArm.getArmPosition());
            System.out.println("Shooter target position: " + armSetpoint);
            System.out.println("Error: " + (armSetpoint - mArm.getArmPosition()));
            System.out.println("*************************** Debug Stats (initialize) ***************************\n");
        }
    }

    @Override
    public void execute() {
        // TODO: try this (constantly calculates the arm setpoint) main fear is that the
        // movement needed is so little that motion magic wont care enough
        if (autoAim) {
            armSetpoint = mArm.calculateArmSetpoint();
        }

        if (Math.abs(Constants.Vision.SpeakerCoords[1]
                - Drivetrain.getInstance().getPose().getY()) < Constants.Vision.SpeakerDeadBand
                && Drivetrain.getInstance().getPose().getX() < 1.91) {
            shootSpeed = 0.6;
        } else {
            shootSpeed = 1;
        }

        // If shooter is empty
        if (!mIntake.getShooterSensor()) {
            // Increased the spead of feeder setVal to 0.5 from 0.1
            mShooter.shoot(0.5, shootSpeed);

            mArm.setMotionMagic(Constants.ArmConstants.SetPoints.kIntake);
            if (!isFinishedBool) {
                SmartDashboard.putString("Shooter/Status", "Waiting for note");
                mIntake.setBelt(Constants.IntakeConstants.beltIntakeSpeed);
            } else {
                SmartDashboard.putString("Shooter/Status", "Returning to Idle");
                mIntake.setBelt(0);
            }
        }

        // else if the arm is within 1.5 degrees of the target and the arm is not moving
        else if (mArm.isInRangeOfTarget(armSetpoint) && Math.abs(mArm.getVelocity()) < 0.5) {
            if (mShooter.readyToShoot()) {
                mShooter.shoot(1, shootSpeed);
                SmartDashboard.putString("Shooter/Status", "Shooting");
                isFinishedBool = true;
            } else {
                mShooter.shoot(0, shootSpeed);
                SmartDashboard.putString("Shooter/Status", "Charging Up");
            }
        }

        // If shooter is loaded but arm is not in position
        else {
            if (timer == null) {
                timer = new Timer();
                timer.restart();
            }

            // reversing the feed and pushing the note out of the shooter to charge up the
            // shooter motors
            if (timer.get() < Constants.ShooterConstants.feederChargeUpTime) {
                mShooter.shoot(Constants.ShooterConstants.feederReverseFeed, 0);
                SmartDashboard.putString("Shooter/Status", "Reversing Feed");
            } else {
                mShooter.shoot(0, shootSpeed);
                mArm.setMotionMagic(armSetpoint);
                SmartDashboard.putString("Shooter/Status", "Traveling to Setpoint");
            }

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
        if (Robot.isSimulation()) {
            return false;
        }

        return isFinishedBool && !mArm.isInRangeOfTarget(armSetpoint, 5);
    }

    @Override
    public void end(boolean interrupted) {
        if (isFinishedBool) {
            mIntake.setHolding(false);
        }

        mArm.setMotionMagic(Constants.ArmConstants.SetPoints.kIntake);

        if (interrupted) {
            SmartDashboard.putString("Shooter/Status", "Aim And Shoot Interrupted");
        }
    }
}