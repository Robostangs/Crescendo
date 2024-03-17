package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class SetPoint extends Command {
    private Arm mArm;
    private double armSetpoint;
    private double error;
    private boolean debugMode = false;
    private boolean autoAim;

    /**
     * <p>Set the shooter to a specific position
     * <p>Only use this if the shooter sensor is not working
     * 
     * @param target in degrees of THE SHOOTER, not the extension bar
     */
    public SetPoint(double target) {
        mArm = Arm.getInstance();
        armSetpoint = target;

        autoAim = false;

        this.setName("Setpoint: " + armSetpoint + " degrees");
        this.addRequirements(mArm);
    }

    /**
     * <p>
     * Set the shooter to a calculated position
     * <p>
     * Only use this if the shooter sensor is not working
     */
    public SetPoint() {
        mArm = Arm.getInstance();

        autoAim = true;

        this.setName("Auto Setpoint");
        this.addRequirements(mArm);
    }

    @Override
    public void initialize() {
        if (autoAim) {
            armSetpoint = mArm.calculateArmSetpoint();
        }

        error = armSetpoint - mArm.getArmPosition();

        if (debugMode) {
            System.out.println("\n*************************** Debug Stats (initialize) ***************************");
            System.out.println("Shooter position: " + mArm.getArmPosition());
            System.out.println("Shooter target position: " + armSetpoint);
            System.out.println("Error: " + error);
            System.out.println("*************************** Debug Stats (initialize) ***************************\n");
        }
        
        mArm.setMotionMagic(armSetpoint);
    }

    @Override
    public void execute() {
        if (autoAim) {
            armSetpoint = mArm.calculateArmSetpoint();
        }

        error = armSetpoint - mArm.getArmPosition();

        // SmartDashboard.putNumber("Arm/Position Error", error);

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
        return false;
    }
}