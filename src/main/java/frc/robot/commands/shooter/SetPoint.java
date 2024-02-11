package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Arm;

public class SetPoint extends Command {
    private final Arm mArm;
    private double armSetpoint;
    private double error = 0;
    private Timer timer;
    private boolean debugMode = false;

    /**
     * Set the shooter to a specific position
     * 
     * @param target in degrees of THE SHOOTER, not the extension bar
     */
    public SetPoint(double target) {
        timer = new Timer();
        mArm = Arm.getInstance();
        armSetpoint = target;

        this.setName("Setpoint: " + armSetpoint + " degrees");
        this.addRequirements(mArm);
    }

    @Override
    public void initialize() {
        timer.restart();

        error = armSetpoint - mArm.getArmPosition();

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

        SmartDashboard.putNumber("Arm/Arm Position Error", error);

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
        if (Robot.isSimulation()) {
            return timer.get() > 1;
        } else {
            return mArm.isInRangeOfTarget(armSetpoint);
        }
    }
}