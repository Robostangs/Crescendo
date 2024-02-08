package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;

public class AimAndShoot extends Command {
    private Arm mArm = Arm.getInstance();
    private Shooter mShooter = Shooter.getInstance();
    private double setpoint;
    private double error = 0;
    private boolean debugMode = false;

    public AimAndShoot() {
        this.setName("Aim and Shoot");
        this.addRequirements(mArm);
        this.addRequirements(mShooter);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        setpoint = mArm.calculateArmSetpoint();

        if (!mArm.validSetpoint(setpoint)) {
            this.end(true);
        }

        // mArm.setArmTarget(setpoint);
        // mShooter.SetRpm(6000, 6000);
        // if (mArm.isInRangeOfTarget(setpoint))
        //     mShooter.loadPiece(true);
        // else
        //     mShooter.loadPiece(false);

        SmartDashboard.putNumber("Arm/Arm Position Error", error);
        SmartDashboard.putNumber("Arm/Velocity", mArm.getVelocity());

        if (debugMode) {
            System.out.println("\n*************************** Debug Stats (execute) ***************************");
            System.out.println("Shooter position: " + mArm.getArmPosition());
            System.out.println("Shooter target position: " + setpoint);
            System.out.println("Error: " + error);
            System.out.println("*************************** Debug Stats (execute) ***************************\n");
        }
    }

    @Override
    public boolean isFinished() {
        // if (Robot.isSimulation()) {
        //     return timer.get() > 0.3;
        // } else {
        //     return mArm.isInRangeOfTarget(armSetpoint);
        // }
        return false;
        // return timer.get() > 0.3;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            SmartDashboard.putNumber("Arm/Velocity", 0);
        }
        // mArm.stop();
        // System.out.println("Shooter position (end of command): " + (mArm.getArmPosition()));
    }
}
