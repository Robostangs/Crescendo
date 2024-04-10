package frc.robot.commands.FeederCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class PassToShooter extends Command {
    Intake intake;
    Shooter shooter;

    /**
     * A command that sets power to the belt, intake wheels, and shooter feeder wheels to pass a note to the shooter
     */
    public PassToShooter() {
        intake = Intake.getInstance();
        shooter = Shooter.getInstance();
        
        this.addRequirements(intake, shooter);
        this.setName("Pass to Shooter");
    }

    @Override
    public void initialize() {
        intake.setBelt(Constants.IntakeConstants.beltIntakeSpeed);
        intake.setIntake(Constants.IntakeConstants.intakeMotorSpeed);
        shooter.setFeederMotor(Constants.ShooterConstants.feederFeedForward);
        intake.postStatus("Passing to Shooter");
        shooter.postStatus("Feeding");
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            intake.postStatus("Passed to Shooter");
            shooter.postStatus("Pass Received");
        }

        else {
            intake.postStatus("Pass Canceled");
            shooter.postStatus("Pass Canceled");
        }

        intake.setBelt(0);
        intake.setIntake(0);
        shooter.setFeederMotor(0);
    }

    @Override
    public boolean isFinished() {
        return intake.getShooterSensor();
    }
}
