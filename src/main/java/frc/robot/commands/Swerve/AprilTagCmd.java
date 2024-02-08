// package frc.robot.Commands.Swerve;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.LimelightHelpers;
// import frc.robot.Subsystems.Swerve.Swerve;
// import frc.robot.Subsystems.Swerve.SwerveModule.DriveRequestType;
// import frc.robot.Subsystems.Swerve.SwerveRequest;

// public class AprilTagCmd extends Command {
//     private Swerve mSwerve = Swerve.getInstance();
//     private Constants.AprilTag target;
//     private SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
//             .withDeadband(Constants.SwerveConstants.kSpeedAt12VoltsMetersPerSecond * 0.08)
//             .withRotationalDeadband(0)
//             .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
//     private PIDController xController, yController, mController;

//     public AprilTagCmd() {
//         this.setName("AprilTagCmd");
//     }

//     @Override
//     public void initialize() {
//         xController = new PIDController(0.2, 0, 0);
//         yController = new PIDController(0.2, 0, 0);
//         mController = new PIDController(
//                 Constants.AutoConstants.translationPID.kP,
//                 Constants.AutoConstants.translationPID.kI,
//                 Constants.AutoConstants.translationPID.kD);

//         // yController.setSetpoint(0);
//         // yController.setTolerance(10);
//     }

//     @Override
//     public void execute() {
//         target = Constants.AprilTag.fromId(LimelightHelpers.getTid(Constants.Vision.llAprilTag));
//         switch (target) {
//             case BlueAmp:
//                 System.out.println("Blue Amp");
//                 break;
//             case BlueCenterStage:
//                 System.out.println("Blue Center Stage");
//                 break;
//             case BlueLeftHumanPlayer:
//                 System.out.println("Blue Left Human Player");
//                 break;
//             case BlueLeftStage:
//                 System.out.println("Blue Left Stage");
//                 break;
//             case BlueRightHumanPlayer:
//                 System.out.println("Blue Right Human Player");
//                 break;
//             case BlueRightStage:
//                 System.out.println("Blue Right Stage");
//                 break;
//             case BlueSpeaker:
//                 // System.out.println("Blue Speaker");

//                 // mSwerve.applyRequest(() -> forwardStraight.withVelocityX(0)
//                 // .withVelocityY(yController.calculate(LimelightHelpers.getTX("limelight"))));

//                 System.out.println(yController.getPositionError());
//                 mSwerve.applyRequest(() -> drive.withVelocityX(xController.calculate(0))
//                         .withVelocityY(yController.calculate(0, LimelightHelpers.getTX("limelight"))));
//                 break;
//             case BlueSpeakerOffset:
//                 System.out.println("Blue Speaker Offset");
//                 break;
//             case NoTag:
//                 System.out.println("No Tag");
//                 break;
//             case RedAmp:
//                 System.out.println("Red Amp");
//                 break;
//             case RedCenterStage:
//                 System.out.println("Red Center Stage");
//                 break;
//             case RedLeftHumanPlayer:
//                 System.out.println("Red Left Human Player");
//                 break;
//             case RedLeftStage:
//                 System.out.println("Red Left Stage");
//                 break;
//             case RedRightHumanPlayer:
//                 System.out.println("Red Right Human Player");
//                 break;
//             case RedRightStage:
//                 System.out.println("Red Right Stage");
//                 break;
//             case RedSpeaker:
//                 System.out.println("Red Speaker");
//                 break;
//             case RedSpeakerOffset:
//                 System.out.println("Red Speaker Offset");
//                 break;
//             default:
//                 System.out.println("No Tag Idenfied");
//                 break;

//         }

//     }

// }