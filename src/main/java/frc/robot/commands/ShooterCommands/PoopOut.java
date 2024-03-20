package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class PoopOut extends Command {
    Shooter shooter;

    public PoopOut() {
        shooter = Shooter.getInstance();

        this.setName("Poop Out Piece");
        this.addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.shoot(Constants.ShooterConstants.feederShootValue, Constants.ShooterConstants.shooterPoopSpeed);
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
