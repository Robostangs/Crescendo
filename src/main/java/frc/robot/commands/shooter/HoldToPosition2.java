// package frc.robot.Commands.Shooter;

// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.Subsystems.Shooter.Arm;

// public class HoldToPosition2 extends Command {
//     private Arm mArm;
//     private double shooterExtensionSetpoint, shooterSetpoint;
//     private double error = 0;
//     private double rateOfMotion = 1.0;
//     private TrapezoidProfile velocityProfile;
//     private TrapezoidProfile.State goalState;
//     private Timer timer;

//     /**
//      * Set the shooter to a specific position
//      * 
//      * @param target in degrees of THE SHOOTER, not the extension bar
//      */
//     public HoldToPosition2(double target) {
//         timer = new Timer();
//         mArm = Arm.getInstance();
//         shooterExtensionSetpoint = target + Constants.ArmConstants.shooterOffset;
//         shooterSetpoint = target;

//         this.setName("Aim for " + shooterSetpoint + " degrees");
//         this.addRequirements(mArm);

//         mArm.setArmTarget(shooterExtensionSetpoint);

//         velocityProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(0.2, 0.2));
//         goalState = new TrapezoidProfile.State(target, 0);
//     }

//     @Override
//     public void initialize() {
//         timer.start();
//         error = shooterExtensionSetpoint - mArm.getShooterExtensionPosition();

//         System.out.println("\n*************************** Debug Stats (initialize) ***************************");
//         System.out.println("Shooter position: " + mArm.getArmPosition());
//         System.out.println("Shooter target position: " + shooterSetpoint);
//         System.out.println("Error: " + error);
//         System.out.println("Output: " + (error * (rateOfMotion / Constants.ArmConstants.kArmRangeOfMotion)));
//         System.out.println("*************************** Debug Stats (initialize) ***************************\n");
//     }

//     @Override
//     public void execute() {
//         if (!mArm.validSetpoint(shooterSetpoint)) {
//             this.end(true);
//         }
//         TrapezoidProfile.State currState = new TrapezoidProfile.State(mArm.getArmPosition(), mArm.getVelocity());

//         TrapezoidProfile.State currentState = velocityProfile.calculate(timer.get(), currState, goalState);

//         error = goalState.position - currentState.position;

//         mArm.aimVelocity(currentState.velocity);

//         // System.out.println("\n*************************** Debug Stats (execute) ***************************");
//         // System.out.println("Shooter position: " + mArm.getArmPosition());
//         // System.out.println("Shooter target position: " + shooterSetpoint);
//         // System.out.println("Error: " + error);
//         // System.out.println("Output: " + (error * (rateOfMotion / Constants.ArmConstants.kArmRangeOfMotion)));
//         // System.out.println("*************************** Debug Stats (execute) ***************************\n");
//     }

//     @Override
//     public boolean isFinished() {
//         return false;
//         // return mArm.isInRangeOfTarget(shooterExtensionSetpoint);
//     }

//     @Override
//     public void end(boolean interrupted) {
//         mArm.stop();
//         System.out.println("Shooter position (end of command): " + (mArm.getArmPosition()));
//     }
// }