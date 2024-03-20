package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class DeployAndIntake extends Command {
    Intake intake;
    Shooter shooter;

    boolean deploy;

    public DeployAndIntake(boolean deploy) {
        this.deploy = deploy;

        intake = Intake.getInstance();
        shooter = Shooter.getInstance();

        this.setName("Deploy And Intake");
        this.addRequirements(intake, shooter);
    }

    @Override
    public void initialize() {
        intake.setExtend(deploy);
        intake.setIntake(Constants.IntakeConstants.intakeMotorSpeed);
        intake.setHolding(true);
        shooter.setFeederMotor(Constants.ShooterConstants.feederFeedForward);
    }

    @Override
    public boolean isFinished() {
        return intake.getShooterSensor();
    }

    @Override
    public void end(boolean interrupted) {
        if (deploy) {
            intake.setExtend(false);
        }

        intake.setIntake(0);
    }
}
