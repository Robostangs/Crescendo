package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Arm;

public class AimAndShootSequential extends ParallelCommandGroup {
    public AimAndShootSequential(double target) {
        addCommands(
            new SetPoint(target),
            new FeedAndShoot(() -> Arm.getInstance().isInRangeOfTarget(target))
        );
    }
}
