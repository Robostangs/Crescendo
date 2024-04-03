package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.ArmCommands.ReturnHome;
import frc.robot.commands.ArmCommands.SetPoint;
import frc.robot.commands.FeederCommands.BeltDrive;
import frc.robot.commands.FeederCommands.PassToShooter;
import frc.robot.commands.ShooterCommands.CancelShooter;
import frc.robot.commands.ShooterCommands.FullSend;
import frc.robot.commands.ShooterCommands.PoopOut;
import frc.robot.commands.ShooterCommands.Prepare;
import frc.robot.commands.ShooterCommands.Shoot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShootCommandFactory {
        // works perfectly
        public static Command getAimAndShootCommand() {
                return new PassToShooter().unless(() -> Intake.getInstance().getShooterSensor())
                                .andThen(new SetPoint()
                                                .raceWith(new WaitUntilCommand(() -> Arm.getInstance().atSetpoint())
                                                                .deadlineWith(new Prepare()).andThen(new Shoot(false))))
                                .withName("Auto Aim and Shoot");
        }

        // works perfectly
        public static Command getAimAndShootCommandWithTimeouts() {
                return new PassToShooter().withTimeout(Constants.OperatorConstants.feedTimeout)
                                .unless(() -> Intake.getInstance().getShooterSensor())
                                .andThen(new ConditionalCommand(
                                                new SetPoint().raceWith(
                                                                new WaitUntilCommand(() -> Arm.getInstance()
                                                                                .atSetpoint())
                                                                                .withTimeout(Constants.OperatorConstants.setpointTimeout)
                                                                                .deadlineWith(new Prepare())
                                                                                .andThen(new Shoot(false)
                                                                                                .withTimeout(Constants.OperatorConstants.shootTimeout),
                                                                                                new Shoot(true).onlyWhile(
                                                                                                                () -> Intake.getInstance()
                                                                                                                                .getShooterSensor()))),
                                                new Spit().withTimeout(Constants.AutoConstants.spitTime),
                                                () -> Intake.getInstance().getShooterSensor()))
                                .withName("Auto Aim and Shoot with Timeouts");

        }

        // works perfectly
        public static Command getAimAndShootCommandWithWaitUntil(BooleanSupplier waitUntil) {
                return new PassToShooter().unless(() -> Intake.getInstance().getShooterSensor())
                                .andThen(new SetPoint()
                                                .raceWith(new Prepare().until(waitUntil).andThen(new Shoot(false))),
                                                new CancelShooter().alongWith(new ReturnHome()))
                                .withName("Aim and Shoot");
        }

        // works perfectly
        public static Command getAmpCommand() {
                return new PassToShooter().unless(() -> Intake.getInstance().getShooterSensor())
                                .andThen(new SetPoint(Constants.ArmConstants.SetPoints.kAmp), new PoopOut(),
                                                new CancelShooter().alongWith(new ReturnHome()))
                                .withName("Auto Amp Shot");
        }

        // works perfectly
        public static Command getAmpCommandWithWaitUntil(BooleanSupplier waitUntil) {
                return new PassToShooter().unless(() -> Intake.getInstance().getShooterSensor()).andThen(
                                new WaitUntilCommand(waitUntil)
                                                .deadlineWith(new SetPoint(Constants.ArmConstants.SetPoints.kAmp)),
                                new PoopOut(),
                                new CancelShooter().alongWith(new ReturnHome())).withName("Amp Shot");
        }

        // works perfectly
        public static Command getPrepareAndShootCommand() {
                return new PassToShooter().unless(() -> Intake.getInstance().getShooterSensor()).andThen(
                                new Prepare().raceWith(new WaitUntilCommand(() -> Shooter.getInstance().readyToShoot()))
                                                .andThen(new Shoot(false)))
                                .withName("Auto Prepare and Shoot");
        }

        // works perfectly
        public static Command getPrepareAndShootCommandWithTimeouts() {
                return new Prepare().until(() -> Shooter.getInstance().readyToShoot())
                                .withTimeout(Constants.OperatorConstants.chargeUpTimeout)
                                .andThen(new Shoot(false).withTimeout(Constants.OperatorConstants.shootTimeout),
                                                new Shoot(true).onlyWhile(
                                                                () -> Intake.getInstance().getShooterSensor()))
                                .withName("Auto Prepare and Shoot with Timeouts");
        }

        // works perfectly
        public static Command getPrepareAndShootCommandWithWaitUntil(BooleanSupplier waitUntil) {
                return new Prepare().until(waitUntil)
                                .andThen(new Shoot(true).onlyWhile(waitUntil))
                                .withName("Prepare and Shoot");
        }

        // works perfectly
        public static Command getRapidFireCommand() {
                return new Shoot(false)
                                .andThen(new BeltDrive(() -> Constants.IntakeConstants.beltIntakeSpeed)
                                                .alongWith(new FullSend()))
                                .withName("Rapid Fire");
        }

        public static Command getCenterToWingCommand(BooleanSupplier waitUntil) {
                return new PassToShooter().unless(() -> Intake.getInstance().getShooterSensor())
                                .andThen(new Prepare().until(waitUntil), new Shoot(true).onlyWhile(waitUntil))
                                .deadlineWith(new SetPoint(Constants.ArmConstants.SetPoints.kCenterToWingPass)
                                                .handleInterrupt(ReturnHome.ReturnHome))
                                .finallyDo(CancelShooter.CancelShooter)
                                .withName("Pass to Center");
                // .andThen(new WaitUntilCommand(waitUntil).deadlineWith(new Prepare(), new
                // SetPoint(Constants.ArmConstants.SetPoints.kCenterToWingPass)))

                // return new PassToShooter().unless(() ->
                // Intake.getInstance().getShooterSensor())
                // .andThen(new Prepare(),
                // new WaitUntilCommand(waitUntil)
                // .deadlineWith(new SetPoint(
                // Constants.ArmConstants.SetPoints.kCenterToWingPass)),
                // new Shoot(false),
                // new CancelShooter().alongWith(new ReturnHome()))
                // .withName("Pass to Center");
        }
}