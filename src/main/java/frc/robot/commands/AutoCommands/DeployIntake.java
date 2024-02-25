package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class DeployIntake extends Command {
    private Intake mIntake;
    private Shooter mShooter;

    // needs to run parallel to the whole auto command
    public DeployIntake() {
        mIntake = Intake.getInstance();
        mShooter = Shooter.getInstance();

        this.setName("Deploy Intake");
        this.addRequirements(mIntake);
    }

    @Override
    public void execute() {
        mIntake.setExtend(true);

        // if (mShooter.getCurrentCommand() != null) {
            // }
            
            // if shooter is empty
            if (!mIntake.getShooterSensor()) {
                mIntake.setIntake(1);
                mIntake.setBelt(0.5);
                mShooter.shoot(-0.2, 0);
        }

        // if shooter has a piece
        else {
            mIntake.setIntake(0);
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        mIntake.setIntake(0);
        mIntake.setBelt(0);
        mIntake.setExtend(false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}