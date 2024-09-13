package frc.Graveyard;
//here lies code that @hellothere smited 
//it wsa kinda the same as prepare soo its not needed
// package frc.robot.commands.ShooterCommands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Shooter;

// public class ChargeUp extends Command {
//     Shooter shooter;
//     double power;

//     /**
//      * Charges shooters to a specified power
//      * @param power percentage to charge it to
//      */
//     public ChargeUp(double power) {
//         shooter = Shooter.getInstance();

//         this.power = power;

//         this.addRequirements(shooter);
//         this.setName("Charge Up");
//     }

//     @Override
//     public void initialize() {
//         shooter.postStatus("Charging to " + power);
//         shooter.setShooterMotors(power);
//     }

//     @Override
//     public boolean isFinished() {
//         return shooter.readyToShoot(power);
//     }
// }
