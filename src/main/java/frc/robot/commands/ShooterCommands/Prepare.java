package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class Prepare extends Command {
    private Shooter shooter;
    private double power;

    /**
     * A command that sets the shooter motors the MAX
     */
    public Prepare() {
        shooter = Shooter.getInstance();
        power = 1;

        this.addRequirements(shooter);
        this.setName("Prepare");
    }
    /**
     * A command that sets the shooter motors to the power that is specified
     * @param power percentage to run it at
     */
    public Prepare(double power) {
        shooter = Shooter.getInstance();
        this.power = power;

        this.addRequirements(shooter);
        this.setName("Prepare");
    }
    @Override
    public void initialize() {
        shooter.setShooterMotors(power);
        shooter.postStatus("Preparing");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
