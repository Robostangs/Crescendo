package frc.robot.commands.shooter;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
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

    private double armSetpoint;
    private double shootSpeed = 1;

    private boolean autoAim = false;
    private boolean isFinishedBool = false;

    private boolean debugMode = false;

    /** when this returns true, this indicates that the user wants to shoot */
    private Supplier<Boolean> chargeUntil = () -> true;

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

    /**
     * Calculates the desired setpoint of the arm using robotPose and then charges
     * the shooter motors. When the user presses the defined button, the shooter
     * will shoot and the command ends
     * 
     * @param chargeUntil the boolean supplier that, when returns true, will shoot
     *                    the piece
     */
    public AimAndShoot(Supplier<Boolean> chargeUntil) {
        mArm = Arm.getInstance();
        mShooter = Shooter.getInstance();
        mIntake = Intake.getInstance();
        isFinishedBool = false;
        autoAim = true;

        this.chargeUntil = chargeUntil;

        this.setName("Auto aim");
        this.addRequirements(mArm, mShooter, mIntake);
    }

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
     * Set the shooter to a specific position and shoots when within 1 degree
     * 
     * @param target      in degrees of THE SHOOTER, not the extension bar
     * @param chargeUntil the boolean supplier that, when returns true, will shoot
     *                    the piece
     */
    public AimAndShoot(double target, Supplier<Boolean> chargeUntil) {
        mArm = Arm.getInstance();
        mShooter = Shooter.getInstance();
        mIntake = Intake.getInstance();
        armSetpoint = target;
        isFinishedBool = false;
        autoAim = false;
        
        this.chargeUntil = chargeUntil;

        this.setName("Aim and Shoot (Charge Until): " + armSetpoint + " degrees");
        this.addRequirements(mArm, mShooter, mIntake);
    }

    @Override
    public void initialize() {
        isFinishedBool = false;

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
        if (autoAim) {
            armSetpoint = mArm.calculateArmSetpoint();
        }

        Pose2d speakerPose;

        if (Robot.isRed()) {
            speakerPose = Constants.Vision.SpeakerPoseRed;
        } else {
            speakerPose = Constants.Vision.SpeakerPoseBlue;
        }

        if (Math.abs(speakerPose.getY()
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
            // TODO: test this
            // if the shooter is ready to shoot and the user has pressed the button
            if ((mShooter.readyToShoot() || shootSpeed == 0.6) && chargeUntil.get()) {
                mShooter.shoot(Constants.ShooterConstants.feederShootValue, shootSpeed);
                SmartDashboard.putString("Shooter/Status", "Shooting");
                isFinishedBool = true;
            }
            
            // if the shooter isnt charge up yet or the user has not said to shoot yet
            else {
                mShooter.shoot(0, shootSpeed);
                SmartDashboard.putString("Shooter/Status", "Charging Up");
            }
        }

        // If shooter is loaded but arm is not in position
        else {
                mArm.setMotionMagic(armSetpoint);
                mShooter.shoot(0, shootSpeed);
                SmartDashboard.putString("Shooter/Status", "Traveling to Setpoint");
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