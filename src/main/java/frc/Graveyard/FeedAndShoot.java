// Here lies @Ardusas mid code
//it served it purpose ok
//???? - 3/27/2024







// package frc.robot.commands.ShooterCommands;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.Shooter;
// import java.util.function.BooleanSupplier;

// public class FeedAndShoot extends Command {
//     private final Shooter mShooter;
//     private final Intake mIntake;
//     private BooleanSupplier feedUntil;

//     /**
//      * @deprecated
//      * This command will wait until a piece is in the shooter and then shoot, but if
//      * there was already a piece in the shooter then it will wait until the shooter
//      * is charged up and then shoot
//      */
//     public FeedAndShoot() {
//         this(null);
//     }

//     /**
//      * This command will wait until a piece is in the shooter and user speciified
//      * condition is met, then shoot
//      * 
//      * @param feedUntil a supplier that returns true when the shooter should shoot,
//      *                  false means to charge up
//      */
//     public FeedAndShoot(BooleanSupplier feedUntil) {
//         mShooter = Shooter.getInstance();
//         mIntake = Intake.getInstance();
//         this.setName("Feed And Shoot");
//         this.addRequirements(mShooter, mIntake);
//         this.feedUntil = feedUntil;
//     }

//     @Override
//     public void initialize() {
//         shooter.postStatus("Charging Up");
//     }

//     @Override
//     public void execute() {
//         if (feedUntil == null) {
//             // if there is a piece in the shooter
//             if (mIntake.getShooterSensor()) {
//                 // if the shooter is already charged up then shoot
//                 if (mShooter.readyToShoot()) {
//                     mShooter.shoot(Constants.ShooterConstants.feederShootValue, 1);
//                     shooter.postStatus("Shooting");
//                     mIntake.setHolding(false);
//                 }

//                 // if the shooter isnt charged up but the pieces has been reversed
//                 else {
//                     mShooter.shoot(0, 1);
//                     shooter.postStatus("Charging Up");
//                 }

//                 mIntake.setBelt(0);
//             }

//             // if there is no piece in the shooter
//             else {
//                 // mShooter.shoot(Constants.ShooterConstants.feederFeedForward, 1);
//                 mShooter.shoot(Constants.ShooterConstants.feederShootValue, 1);
//                 mIntake.setBelt(Constants.IntakeConstants.beltIntakeSpeed);
//                 shooter.postStatus("Waiting for piece");
//             }
//         }

//         // if we are waiting for user to press shoot button
//         else {
//             // if there is a piece in the shooter
//             if (mIntake.getShooterSensor()) {
//                 // if the shooter is ready to shoot and the user is ready then shoot
//                 if (feedUntil.getAsBoolean() && mShooter.readyToShoot()) {
//                     mShooter.shoot(Constants.ShooterConstants.feederShootValue, 1);
//                     shooter.postStatus("Shooting");
//                     mIntake.setHolding(false);
//                 }

//                 // if the shooter isnt charged up but the pieces has been reversed
//                 else {
//                     mShooter.shoot(0, 1);
//                     shooter.postStatus("Charging Up");
//                 }

//                 mIntake.setBelt(0);
//             }

//             // if there is no piece in the shooter
//             else {
//                 mShooter.shoot(Constants.ShooterConstants.feederFeedForward, 1);
//                 mIntake.setBelt(Constants.IntakeConstants.beltIntakeSpeed);
//                 shooter.postStatus("Waiting for piece");
//             }
//         }
//     }

//     @Override
//     public void end(boolean interrupted) {
//         shooter.postStatus("Idle");
//     }

//     @Override
//     public boolean isFinished() {
//         return false;
//     }
// }
