package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class HoldToPosition extends Command {
    private Arm mArm;
    private double shooterExtensionSetpoint, shooterSetpoint;
    private double error = 0;
    private double rateOfMotion = 1.0;

    /**
     * Set the shooter to a specific position
     * 
     * @param target in degrees of THE SHOOTER, not the extension bar
     */
    public HoldToPosition(double target) {
        mArm = Arm.getInstance();
        shooterExtensionSetpoint = target + Constants.ArmConstants.shooterOffset;
        shooterSetpoint = target;

        this.setName("Aim for " + shooterSetpoint + " degrees");
        this.addRequirements(mArm);
    }

    @Override
    public void initialize() {
        error = shooterExtensionSetpoint - mArm.getShooterExtensionPosition();

        System.out.println("\n*************************** Debug Stats (initialize) ***************************");
        System.out.println("Shooter position: " + mArm.getArmPosition());
        System.out.println("Shooter target position: " + shooterSetpoint);
        System.out.println("Error: " + error);
        System.out.println("Output: " + (error * (rateOfMotion / Constants.ArmConstants.kArmRangeOfMotion)));
        System.out.println("*************************** Debug Stats (initialize) ***************************\n");
    }

    @Override
    public void execute() {
        if (!mArm.validSetpoint(shooterSetpoint)) {
            this.end(true);
        }

        error = shooterExtensionSetpoint - mArm.getShooterExtensionPosition();

        if (Math.abs(error) > 0.05) {
            mArm.aimRaw(error * (rateOfMotion / Constants.ArmConstants.kArmRangeOfMotion));
        } else {
            end(false);
        }

        mArm.setArmTarget(shooterExtensionSetpoint);

        // if (DriverStation.isTest()) {
            System.out.println("\n*************************** Debug Stats (execute) ***************************");
            System.out.println("Shooter position: " + mArm.getArmPosition());
            System.out.println("Shooter target position: " + shooterSetpoint);
            System.out.println("Error: " + error);
            System.out.println("Output: " + (error * (rateOfMotion / Constants.ArmConstants.kArmRangeOfMotion)));
            System.out.println("*************************** Debug Stats (execute) ***************************\n");
        // }
    }

    @Override
    public boolean isFinished() {
        return false;
        // return mArm.isInRangeOfTarget(shooterExtensionSetpoint);
    }

    @Override
    public void end(boolean interrupted) {
        // mArm.stop();
        System.out.println("Shooter position (end of command): " + (mArm.getArmPosition()));
    }
}