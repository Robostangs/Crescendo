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

public class ShootCommandFactory {
        public static Command getAimAndShootCommand() {
                return new PassToShooter().unless(() -> Intake.getInstance().getShooterSensor())
                                .andThen(new SetPoint()
                                                .raceWith(new WaitUntilCommand(() -> Arm.getInstance().atSetpoint())
                                                                .deadlineWith(new Prepare()).andThen(new Shoot(false))))
                                .withName("Auto Aim and Shoot");
        }

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

        public static Command getAimAndShootCommandWithWaitUntil(BooleanSupplier waitUntil) {
                return new PassToShooter().unless(() -> Intake.getInstance().getShooterSensor())
                                .andThen(new SetPoint()
                                                .raceWith(new Prepare().alongWith(new WaitUntilCommand(waitUntil))
                                                                .andThen(new Shoot(false))))
                                .handleInterrupt(CancelShooter.cancelShooter)

                                // new CancelShooter().alongWith(new ReturnHome()))
                                // new Prepare().until(waitUntil).andThen(new Shoot(false))
                                // .raceWith(new WaitUntilCommand(waitUntil))
                                // .andThen(new WaitUntilCommand(waitUntil),
                                // new Shoot(false))
                                // .deadlineWith(new SetPoint()))
                                .withName("Aim and Shoot");
        }

        public static Command getAmpCommand() {
                return new PassToShooter().unless(() -> Intake.getInstance().getShooterSensor())
                                .andThen(new SetPoint(Constants.ArmConstants.SetPoints.kAmp), new PoopOut())
                                .finallyDo(ReturnHome.returnHome)
                                // new CancelShooter().alongWith(new ReturnHome()))
                                .withName("Auto Amp Shot");
        }

        public static Command getAmpCommandWithWaitUntil(BooleanSupplier waitUntil) {
                return new PassToShooter().unless(() -> Intake.getInstance().getShooterSensor()).andThen(
                                new WaitUntilCommand(waitUntil)
                                                .deadlineWith(new SetPoint(Constants.ArmConstants.SetPoints.kAmp)),
                                new PoopOut()).finallyDo(ReturnHome.returnHome)
                                // new CancelShooter().alongWith(new ReturnHome()))
                                .withName("Amp Shot");
        }

        public static Command getPrepareAndShootCommand() {
                return new Shoot(false).withName("Auto Prepare and Shoot");
        }

        public static Command getPrepareAndShootCommandWithTimeouts() {
                return new Shoot(false).withTimeout(Constants.OperatorConstants.chargeUpTimeout)
                                .andThen(new Shoot(true).onlyWhile(() -> Intake.getInstance().getShooterSensor()))
                                .withName("Auto Prepare and Shoot with Timeouts");

                // return new Prepare()
                // .withTimeout(Constants.OperatorConstants.chargeUpTimeout)
                // .andThen(new
                // Shoot(false).withTimeout(Constants.OperatorConstants.shootTimeout),
                // new Shoot(true).onlyWhile(
                // () -> Intake.getInstance().getShooterSensor()))
                // .withName("Auto Prepare and Shoot with Timeouts");
        }

        public static Command getPrepareAndShootCommandWithWaitUntil(BooleanSupplier waitUntil) {
                return new Prepare().alongWith(new WaitUntilCommand(waitUntil))
                                .andThen(new Shoot(true).onlyWhile(waitUntil))
                                .finallyDo(CancelShooter.cancelShooter)
                                .withName("Prepare and Shoot");
        }

        public static Command getRapidFireCommand() {
                return new Shoot(false)
                                .andThen(new BeltDrive(() -> Constants.IntakeConstants.beltIntakeSpeed)
                                                .alongWith(new FullSend()))
                                .withName("Auto Rapid Fire");
        }

        public static Command getRapidFireCommandWithWaitUntil(BooleanSupplier waitUntil) {
                return new WaitUntilCommand(waitUntil).deadlineWith(new Prepare())
                                // Prepare().until(waitUntil)
                                .andThen(new BeltDrive(() -> Constants.IntakeConstants.beltIntakeSpeed)
                                                .alongWith(new FullSend()).onlyWhile(waitUntil))
                                .finallyDo(CancelShooter.cancelShooter)
                                .withName("Rapid Fire");
        }

        public static Command getCenterToWingCommand(BooleanSupplier waitUntil) {
                return new PassToShooter().unless(() -> Intake.getInstance().getShooterSensor()).andThen(
                                new WaitUntilCommand(waitUntil)
                                                .deadlineWith(new SetPoint(
                                                                Constants.ArmConstants.SetPoints.kCenterToWingPass)),
                                new Shoot(false)).finallyDo(ReturnHome.returnHome)
                                .handleInterrupt(CancelShooter.cancelShooter)

                                // new CancelShooter().alongWith(new ReturnHome()))
                                .withName("Pass to Center")
                                .finallyDo(ReturnHome.returnHome);
        }
}