package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;

import java.util.function.BooleanSupplier;

public class AimAndShoot extends Command {
    private Arm mArm;
    private Shooter mShooter;
    private double armSetpoint;
    private double error = 0;
    private Timer timer;
    private boolean debugMode = false;
    private BooleanSupplier atPosition;

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
        this.atPosition = () -> true;

        this.setName("Setpoint: " + armSetpoint + " degrees");
        this.addRequirements(mArm, mShooter);
    }

    public AimAndShoot(double target, BooleanSupplier atPosition) {
        timer = new Timer();
        mArm = Arm.getInstance();
        mShooter = Shooter.getInstance();
        armSetpoint = target;
        this.atPosition = atPosition;

        this.setName("Setpoint: " + armSetpoint + " degrees");
        this.addRequirements(mArm, mShooter);
    }

    @Override
    public void initialize() {
        timer.restart();

        SmartDashboard.putString("Shooter/Shooter State", "Charging Up");

        mShooter.setHolding(true);

        error = armSetpoint - mArm.getArmPosition();
        SmartDashboard.putNumber("Arm/Arm Position Error", error);
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

        if (error < 1 && timer.get() > Constants.ShooterConstants.shooterChargeUpTime && atPosition.getAsBoolean()) {
            mShooter.shoot(1, 0.95, 1);
            SmartDashboard.putString("Shooter/Shooter State", "Shooting");
        } else {
            mShooter.shoot(Constants.ShooterConstants.kFeederFeedForward, 1);
            SmartDashboard.putString("Shooter/Shooter State", "Charging Up");
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
        // return false;

        if (Robot.isSimulation()) {
            return timer.get() > 1;
        } else {
            /* TODO: this dont work fix it */
            return mArm.isInRangeOfTarget(armSetpoint);
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            SmartDashboard.putNumber("Constants2.", 0);
        }
        SmartDashboard.putString("Shooter/Shooter State", "Idle");
        mShooter.setBrakeMode(true);
    }
}