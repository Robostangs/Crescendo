package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;

public class DeployAndIntake extends Command {
    private Intake mIntake;
    private Arm mArm;
    private Timer timer;
    private boolean deploy;

    public DeployAndIntake(boolean deploy) {
        mIntake = Intake.getInstance();
        mArm = Arm.getInstance();

        this.deploy = deploy;

        timer = new Timer();
        this.setName("Deploy and Intake");
        this.addRequirements(mIntake, mArm);
    }

    @Override
    public void initialize() {
        timer.restart();
        mArm.setMotionMagic(Constants.ArmConstants.SetPoints.kIntake);
    }

    @Override
    public void execute() {
        if (deploy) {
            mIntake.setExtend(true);
            if (timer.get() > Constants.IntakeConstants.kDeployTimeSeconds) {
                mIntake.setIntake(1);
            }
        } else {
            mIntake.setBelt(1);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
        // return mIntake.getBeltSensor();
    }

    @Override
    public void end(boolean interrupted) {
        mIntake.setIntake(0);
        mIntake.setBelt(0);
        mIntake.setExtend(false);
    }
}
