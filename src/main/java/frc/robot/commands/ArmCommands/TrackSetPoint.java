package frc.robot.commands.ArmCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class TrackSetPoint extends Command {
    private Arm mArm;
    private Supplier<Double> armSetpoint;
    private double error;
    private boolean debugMode = false;

    /**
     * <p>Set the shooter to a specific position
     * <p>Only use this if the shooter sensor is not working
     * 
     * @param target in degrees of THE SHOOTER, not the extension bar
     */
    public TrackSetPoint(Supplier<Double> target) {
        mArm = Arm.getInstance();
        armSetpoint = target;

        this.setName("Track Setpoint");
        this.addRequirements(mArm);
    }

    @Override
    public void initialize() {
        error = armSetpoint.get() - mArm.getArmPosition();

        if (debugMode) {
            System.out.println("\n*************************** Debug Stats (initialize) ***************************");
            System.out.println("Shooter position: " + mArm.getArmPosition());
            System.out.println("Shooter target position: " + armSetpoint);
            System.out.println("Error: " + error);
            System.out.println("*************************** Debug Stats (initialize) ***************************\n");
        }
        
        mArm.setMotionMagic(armSetpoint.get());
    }

    @Override
    public void execute() {
        error = armSetpoint.get() - mArm.getArmPosition();
        mArm.setMotionMagic(armSetpoint.get());

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