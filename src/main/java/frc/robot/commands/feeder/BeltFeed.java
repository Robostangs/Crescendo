// package frc.robot.commands.feeder;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Intake;

// public class BeltFeed extends Command {
//     private Intake mIntake;

//     public BeltFeed() {
//         mIntake = Intake.getInstance();
//         this.setName("Belt Feed");
//         this.addRequirements(mIntake);
//     }

//     @Override
//     public void execute() {
//         if (mIntake.getExtended()) {
//             mIntake.setMotor(1);
//         }
//     }
// }
