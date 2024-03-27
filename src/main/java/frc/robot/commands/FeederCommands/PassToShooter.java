package frc.robot.commands.FeederCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class PassToShooter extends Command {
    Intake intake;
    Shooter shooter;

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
    }

    @Override
    public void end(boolean interrupted) {
        intake.setBelt(0);
        intake.setIntake(0);
        shooter.setFeederMotor(0);
    }

    @Override
    public boolean isFinished() {
        return intake.getShooterSensor();
    }
}
