package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;

public class FeedAndShoot extends Command {
    private final Shooter shooter;
    private Timer timer;

    public FeedAndShoot() {
        timer = new Timer();
        shooter = Shooter.getInstance();
        this.setName("Shoot");
        this.addRequirements(shooter);
    }

    @Override
    public void initialize() {
        timer.restart();
        Arm.getInstance().setBrake(true);
    }

    @Override
    public void execute() {
        if (timer.get() < 2) {
            shooter.shoot(1, 0.0001);
        } else {
            shooter.shoot(1, 1);
        }
        // if (timer.get() < 0.4) {
        //     shooter.shoot(0, 0.3);
        // } else {
        //     shooter.shoot(01, 0);
        // }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setBrakeMode(true);
        Arm.getInstance().setBrake(false);
        shooter.stop();

        if (!(timer.get() < 0.3)) {
            shooter.toggleHolding();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
