package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmCommands.SetPoint;
import frc.robot.commands.FeederCommands.BeltDrive;
import frc.robot.commands.FeederCommands.PassToShooter;
import frc.robot.commands.ShooterCommands.CancelShooter;
import frc.robot.commands.ShooterCommands.FullSend;
import frc.robot.commands.ShooterCommands.PoopOut;
import frc.robot.commands.ShooterCommands.Prepare;
import frc.robot.commands.ShooterCommands.Shoot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;

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

        return new PassToShooter().andThen(new WaitUntilCommand(() -> Arm.getInstance().atSetpoint())
                .deadlineWith(new Prepare()).andThen(new Shoot()).deadlineWith(new SetPoint()))
                .withName("Auto Aim and Shoot");
    }

    public static Command getAimAndShootCommandWithWaitUntil() {
        configureSticks();

        return new PassToShooter().andThen(new WaitUntilCommand(() -> Arm.getInstance().atSetpoint())
                .deadlineWith(new Prepare())
                .andThen(new WaitUntilCommand(() -> xManip.getHID().getLeftBumper()), new Shoot())
                .deadlineWith(new SetPoint())).withName("Aim and Shoot");
    }

    public static Command getAmpCommand() {
        configureSticks();

        return new PassToShooter().andThen(new SetPoint(Constants.ArmConstants.SetPoints.kAmp), new PoopOut())
                .withName("Auto Amp Shot");
    }

    public static Command getAmpCommandWithWaitUntil() {
        configureSticks();

        return new PassToShooter().andThen(
                new WaitUntilCommand(() -> xManip.getHID().getLeftBumper())
                        .deadlineWith(new SetPoint(Constants.ArmConstants.SetPoints.kAmp)),
                new PoopOut(),
                new CancelShooter()).withName("Amp Shot");
    }

    public static Command getPrepareAndShootCommand() {
        configureSticks();

        return new Prepare().raceWith(new WaitUntilCommand(() -> Shooter.getInstance().readyToShoot()))
                .andThen(new Shoot()).withName("Auto Prepare and Shoot");
    }

    public static Command getPrepareAndShootCommandWithWaitUntil() {
        configureSticks();

        return new Prepare().raceWith(new WaitUntilCommand(() -> xManip.getHID().getLeftBumper())).andThen(new Shoot())
                .withName("Prepare and Shoot");
    }

    public static Command getRapidFireCommand() {
        configureSticks();

        return new Shoot()
                .andThen(new BeltDrive(() -> Constants.IntakeConstants.beltIntakeSpeed).alongWith(new FullSend()))
                .withName("Rapid Fire");
    }

    public static Command getCenterToWingCommand() {
        configureSticks();

        return new PassToShooter().andThen(
                new WaitUntilCommand(() -> xManip.getHID().getLeftBumper())
                        .deadlineWith(new SetPoint(Constants.ArmConstants.SetPoints.kCenterToWingPass)),
                new Shoot(),
                new CancelShooter()).withName("Pass to Center");
    }
}