package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmCommands.SetPoint;
import frc.robot.commands.FeederCommands.PassToShooter;
import frc.robot.commands.ShooterCommands.PoopOut;
import frc.robot.commands.ShooterCommands.Prepare;
import frc.robot.commands.ShooterCommands.Shoot;
import frc.robot.subsystems.Arm;

public class ShootCommandFactory {
    static boolean configured;

    static CommandXboxController xDrive, xManip;

    public static void configureSticks() {
        if (!configured) {
            configured = true;
            xDrive = RobotContainer.xDrive;
            xManip = RobotContainer.xManip;
        }
    }

    public static Command getAimAndShootCommand() {
        configureSticks();

        return new PassToShooter().andThen(new SetPoint()
                .raceWith(
                        new WaitUntilCommand(() -> Arm.getInstance().atSetpoint()).deadlineWith(new Prepare()).andThen(
                                new Shoot())));
        // new ReturnToHome());
    }

    // TODO: might be able to remove the conditional
    public static Command getAimAndShootCommandWithWaitUntil() {
        configureSticks();

        return new PassToShooter().andThen(new SetPoint().alongWith(new RepeatCommand(
                new ConditionalCommand(new Shoot(), new Prepare(), () -> xManip.getHID().getLeftBumper()))));
        // , new ReturnToHome());
    }

    public static Command getAmpCommand() {
        configureSticks();

        return new PassToShooter().andThen(new SetPoint(Constants.ArmConstants.SetPoints.kAmp), new PoopOut());
        // , new ReturnToHome());
    }

    
    public static Command getAmpCommandWithWaitUntil() {
        configureSticks();

        return new PassToShooter().andThen(new SetPoint(Constants.ArmConstants.SetPoints.kAmp),
                new WaitUntilCommand(() -> xManip.getHID().getLeftBumper()), new PoopOut());
        // , new ReturnToHome());
    }

    public static Command prepareAndShoot() {
        configureSticks();

        return new Prepare().andThen(new Shoot());
    }

    // TODO: might be able to remove the conditional
    public static Command prepareAndShootWithWaitUntil() {
        configureSticks();

        return new RepeatCommand(
                new ConditionalCommand(new Shoot(), new Prepare(), () -> xManip.getHID().getLeftBumper()));
    }
}