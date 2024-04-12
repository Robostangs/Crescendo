package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ChargethenShoot extends Command {
    Shooter shooter;
    double power;

 
    public ChargethenShoot(double power) {
        shooter = Shooter.getInstance();

        this.power = power;

        this.addRequirements(shooter);
        this.setName("Auto Charge Up");
    }

    @Override
    public void execute() {
        shooter.postStatus("Charging the shooter to: " + power);
        shooter.setShooterMotors(power);
    }

    @Override
    public boolean isFinished() {
        return shooter.readyToShoot(power);
    }

}
