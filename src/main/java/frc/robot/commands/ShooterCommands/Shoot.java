package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {
    Shooter shooter;

    public Shoot() {
        shooter = Shooter.getInstance();
    }

    @Override
    public void initialize() {
        shooter.setShooterMotors(1);
    }

    @Override
    public void execute() {
        if (shooter.readyToShoot()) {
            shooter.setFeederMotor(1);
        }
    }

    @Override
    public boolean isFinished() {
        return !Intake.getInstance().getShooterSensor();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShooterMotors(0);
        shooter.setFeederMotor(0);
    }    
}