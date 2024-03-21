package frc.robot.commands.ArmCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class TrackSetPoint extends Command {
    Arm mArm;
    Supplier<Double> armSetpoint;

    public TrackSetPoint(Supplier<Double> target) {
        mArm = Arm.getInstance();
        armSetpoint = target;

        this.addRequirements(mArm);
        this.setName("Track Setpoint");
    }

    @Override
    public void initialize() {
        mArm.setMotionMagic(armSetpoint.get());
    }

    @Override
    public void execute() {
        mArm.setMotionMagic(armSetpoint.get());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}