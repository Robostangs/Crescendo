package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class DeployAndIntake extends Command {
    Intake intake;
    Shooter shooter;

    boolean deploy;

    /**
     * A command that sets power to the belt and deploys the intake
     * @param deploy If the crashbar should deploy or not 
     */
    public DeployAndIntake(boolean deploy) {
        this.deploy = deploy;

        intake = Intake.getInstance();
        shooter = Shooter.getInstance();

        this.addRequirements(intake, shooter);
        this.setName("Deploy And Intake");
    }

    @Override
    public void initialize() {
        if (deploy) {
            intake.setExtend(true);
            intake.setIntake(Constants.IntakeConstants.intakeMotorSpeed);
            intake.postStatus("Deploying Intake");
        }

        else {
            intake.postStatus("Intaking");
            shooter.postStatus("Feeding");
        }

        intake.setBelt(Constants.IntakeConstants.beltIntakeSpeed);
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
            intake.setIntake(0);
        }
        shooter.setFeederMotor(0);
        intake.setBelt(0);
    }
}
