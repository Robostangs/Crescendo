package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Arm;

public class AimAndShoot extends Command {
    private Arm mArm;
    private double setpoint;
    private double error = 0;
    private boolean debugMode = false;
    private Timer timer;

    public AimAndShoot() {
        mArm = Arm.getInstance();

        this.setName("Aim and Shoot");
        this.addRequirements(mArm);

        timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        setpoint = mArm.calculateArmSetpoint();

        if (!mArm.validSetpoint(setpoint)) {
            this.end(true);
        }

        mArm.setMotionMagic(setpoint);
        error = setpoint - mArm.getArmPosition();

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

    // @Override
    // public boolean isFinished() {
    //     // if (Robot.isSimulation()) {
    //     //     return timer.get() > 0.3;
    //     // } else {
    //     //     return mArm.isInRangeOfTarget(armSetpoint);
    //     // }
    //     return false;
    //     // return timer.get() > 0.3;
    // }

    @Override
    public boolean isFinished() {
        return timer.get() > 5;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            SmartDashboard.putNumber("Arm/Velocity", 0);
        }
        mArm.stop();
        // System.out.println("Shooter position (end of command): " + (mArm.getArmPosition()));
    }
}