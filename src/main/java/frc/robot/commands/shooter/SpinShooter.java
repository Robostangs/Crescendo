package frc.robot.commands.shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SpinShooter extends Command {
    Shooter mShooter = Shooter.getInstance();

    private DoubleSupplier left, right;
    private BooleanSupplier feed;

    public SpinShooter(DoubleSupplier left, DoubleSupplier right, BooleanSupplier feed) {
        this.left = left;
        this.right = right;
        this.feed = feed;
        addRequirements(mShooter);
    }

    @Override
    public void execute() {
        mShooter.shoot(left.getAsDouble(), right.getAsDouble());
        // mShooter.loadPiece(feed.getAsBoolean());
    }
}
