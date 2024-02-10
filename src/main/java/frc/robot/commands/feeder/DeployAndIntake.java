package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;

public class DeployAndIntake extends Command {
    private Intake mIntake;
    private Timer timer;

    public DeployAndIntake() {
        mIntake = Intake.getInstance();
        timer = new Timer();
        this.setName("Deploy and Intake");
        this.addRequirements(mIntake);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        mIntake.setExtend(true);
        if (timer.get() > Constants.IntakeConstants.kDeployTimeSeconds) {
            mIntake.setMotor(1);
        }
    }

    @Override
    public void end(boolean interrupted) {
        mIntake.setMotor(0);
        if (!interrupted) {
        }
        mIntake.setExtend(false);
    }
}
