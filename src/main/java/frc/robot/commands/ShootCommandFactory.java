package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
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

                // return new PassToShooter().andThen(
                // new SetPoint().raceWith(
                // new WaitUntilCommand(() -> Arm.getInstance().atSetpoint()).deadlineWith(new
                // Prepare())),
                // new Shoot(false)).withName("Auto Aim and Shoot");

                return new PassToShooter().onlyIf(() -> !Intake.getInstance().getShooterSensor())
                                .andThen(new SetPoint()
                                                .raceWith(new WaitUntilCommand(() -> Arm.getInstance().atSetpoint())
                                                                .deadlineWith(new Prepare()).andThen(new Shoot(false))))
                                .withName("Auto Aim and Shoot");

                // return new PassToShooter().andThen(new WaitUntilCommand(() ->
                // Arm.getInstance().atSetpoint())
                // .deadlineWith(new Prepare()).andThen(new Shoot(false)).deadlineWith(new
                // SetPoint()))
                // .withName("Auto Aim and Shoot");
        }

        public static Command getAimAndShootCommandWithTimeouts() {
                configureSticks();

                // return new
                // PassToShooter().withTimeout(Constants.OperatorConstants.feedTimeout)
                // .andThen(new SetPoint().raceWith(new WaitUntilCommand(
                // () -> Arm.getInstance().atSetpoint())
                // .withTimeout(Constants.OperatorConstants.setpointTimeout)
                // .deadlineWith(new Prepare())
                // .andThen(new Shoot(false)
                // .withTimeout(Constants.OperatorConstants.shootTimeout)
                // .onlyIf(() -> Intake.getInstance().getShooterSensor()),
                // new Spit().withTimeout(Constants.AutoConstants.spitTime))))
                // .withName("Auto Aim and Shoot with Timeouts");

                return new PassToShooter().onlyIf(() -> !Intake.getInstance().getShooterSensor())
                                .withTimeout(Constants.OperatorConstants.feedTimeout)
                                .andThen(new ConditionalCommand(new SetPoint().raceWith(new WaitUntilCommand(
                                                () -> Arm.getInstance().atSetpoint())
                                                .withTimeout(Constants.OperatorConstants.setpointTimeout)
                                                .deadlineWith(new Prepare())
                                                .andThen(new Shoot(false)
                                                                .withTimeout(Constants.OperatorConstants.shootTimeout))),
                                                new Spit().withTimeout(Constants.AutoConstants.spitTime),
                                                () -> Intake.getInstance().getShooterSensor()))
                                .withName("Auto Aim and Shoot with Timeouts");

        }

        public static Command getAimAndShootCommandWithWaitUntil() {
                configureSticks();

                return new PassToShooter().onlyIf(() -> !Intake.getInstance().getShooterSensor())
                                .andThen(new WaitUntilCommand(() -> Arm.getInstance().atSetpoint())
                                                .deadlineWith(new Prepare())
                                                .raceWith(new WaitUntilCommand(() -> xManip.getHID().getLeftBumper()))
                                                .andThen(new WaitUntilCommand(() -> xManip.getHID().getLeftBumper()),
                                                                new Shoot(false))
                                                .deadlineWith(new SetPoint()))
                                .withName("Aim and Shoot");
        }

        public static Command getAmpCommand() {
                configureSticks();

                return new PassToShooter().onlyIf(() -> !Intake.getInstance().getShooterSensor())
                                .andThen(new SetPoint(Constants.ArmConstants.SetPoints.kAmp), new PoopOut(),
                                                new CancelShooter().alongWith(new ReturnHome()))
                                .withName("Auto Amp Shot");
        }

        public static Command getAmpCommandWithWaitUntil() {
                configureSticks();

                return new PassToShooter().onlyIf(() -> !Intake.getInstance().getShooterSensor()).andThen(
                                new WaitUntilCommand(() -> xManip.getHID().getLeftBumper())
                                                .deadlineWith(new SetPoint(Constants.ArmConstants.SetPoints.kAmp)),
                                new PoopOut(),
                                new CancelShooter().alongWith(new ReturnHome())).withName("Amp Shot");
        }

        public static Command getPrepareAndShootCommand() {
                configureSticks();

                return new Prepare().raceWith(new WaitUntilCommand(() -> Shooter.getInstance().readyToShoot()))
                                .andThen(new Shoot(false)).withName("Auto Prepare and Shoot");
        }

        public static Command getPrepareAndShootCommandWithTimeouts() {
                configureSticks();

                return new Prepare().raceWith(new WaitUntilCommand(() -> Shooter.getInstance().readyToShoot()))
                                .withTimeout(Constants.OperatorConstants.chargeUpTimeout)
                                .andThen(new Shoot(false).withTimeout(Constants.OperatorConstants.shootTimeout),
                                                new Spit().withTimeout(Constants.AutoConstants.spitTime))
                                .withName("Auto Prepare and Shoot with Timeouts");
        }

        public static Command getPrepareAndShootCommandWithWaitUntil() {
                configureSticks();

                return new WaitUntilCommand(() -> xManip.getHID().getLeftBumper()).deadlineWith(new Prepare())
                                .andThen(new Shoot(true).until(() -> !xManip.getHID().getLeftBumper()))
                                .withName("Prepare and Shoot");
        }

        public static Command getRapidFireCommand() {
                configureSticks();

                return new Shoot(false)
                                .andThen(new BeltDrive(() -> Constants.IntakeConstants.beltIntakeSpeed)
                                                .alongWith(new FullSend()))
                                .withName("Rapid Fire");
        }

        public static Command getCenterToWingCommand() {
                configureSticks();

                return new PassToShooter().onlyIf(() -> !Intake.getInstance().getShooterSensor()).andThen(
                                new WaitUntilCommand(() -> xManip.getHID().getLeftBumper())
                                                .deadlineWith(new SetPoint(
                                                                Constants.ArmConstants.SetPoints.kCenterToWingPass)),
                                new Shoot(false),
                                new CancelShooter()).withName("Pass to Center");
        }
}