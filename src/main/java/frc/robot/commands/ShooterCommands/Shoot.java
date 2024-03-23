package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {
    Shooter shooter;
    boolean force;

    public Shoot(boolean force) {
        shooter = Shooter.getInstance();
        this.force = force;

        this.addRequirements(shooter);
        this.setName("Shoot");
    }

    @Override
    public void initialize() {
        shooter.setShooterMotors(1);
        shooter.setFeederMotor(0);
    }

    @Override
    public void execute() {
        if (force || shooter.readyToShoot()) {
            shooter.setFeederMotor(1);
        }
    }

    @Override
    public boolean isFinished() {
        if (force) {
            return false;
        }
        
        return !Intake.getInstance().getShooterSensor();
    }

    @Override
    public void end(boolean interrupted) {
        Intake.getInstance().setHolding(false);
        shooter.setShooterMotors(0);
        shooter.setFeederMotor(0);
    }    
}