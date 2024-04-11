package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class FullSend extends Command {
    Shooter shooter;
    double power;

    public FullSend() {
        power = 0;
        shooter = Shooter.getInstance();

        this.addRequirements(shooter);
        this.setName("Full Send");
    }


    public FullSend(double power) {
        this.power = power;
        shooter = Shooter.getInstance();

        this.addRequirements(shooter);
        this.setName("Kinda Send");
    }

    @Override
    public void initialize() {

        if(power == 0){
        shooter.postStatus("SEND IT");
        shooter.shoot(1, 1);
        }
        else{
            shooter.postStatus("WE ARE SENDING IT");
            shooter.shoot(Constants.ShooterConstants.feederShootValue, power);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.shoot(0, 0);
    }
}
