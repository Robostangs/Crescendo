package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class PoopOut extends Command {
    Shooter shooter;

    /**
     * we all have to go this is just how our robot does it
     * <p>
     * sets shooter and feeder wheels to the max until the note is pooped out
     */
    public PoopOut() {
        shooter = Shooter.getInstance();

        this.addRequirements(shooter);
        this.setName("Poop Out Piece");
    }

    @Override
    public void execute() {
        shooter.shoot(Constants.ShooterConstants.feederShootValue, Constants.ShooterConstants.shooterPoopSpeed);
        shooter.postStatus("Pooping");
    }

    @Override
    public boolean isFinished() {
        return !Intake.getInstance().getShooterSensor();
    }

    @Override
    public void end(boolean interrupted) {
        Intake.getInstance().setHolding(false);
        shooter.setShooterMotors(0);
        shooter.setFeederMotor(0);
    }
}
