package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class FullSend extends Command {
    Shooter shooter;
    double power;

    /**
     * A command that sets the shooter motors TO THE MAX
     */
    public FullSend() {
        power = 1;
        shooter = Shooter.getInstance();

        this.addRequirements(shooter);
        this.setName("Full Send");
    }
   /**
     * A command that sets the shooter motors to the spsecified power
     * @param power percentage to run it at
     */
    public FullSend(double power) {
        this.power = power;
        shooter = Shooter.getInstance();

        this.addRequirements(shooter);
        this.setName("Kinda Send " + power * 100 + "%");
    }

    @Override
    public void initialize() {
        shooter.postStatus("SEND IT");
        shooter.shoot(Constants.ShooterConstants.feederShootValue, power);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.shoot(0, 0);
    }
}
