package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.subsystems.Drivetrain.SwerveRequest;

import java.util.function.Supplier;

public class xDrive extends Command {
    private final Drivetrain drivetrain;
    private Supplier<Double> translateX, translateY, rotate, howManyBabiesOnBoard;
    private SwerveRequest swerveRequest;

    /**
     * Command to set the drivetrain to brake mode when not moving
     * 
     * @param translateX           the forward to backward movement of the robot
     * @param translateY           the right to left movement of the robot
     * @param rotate               the rotational movement of the robot
     * @param howManyBabiesOnBoard 1 - the value of how much to slow down (right
     *                             trigger axis)
     */
    public xDrive(Supplier<Double> translateX, Supplier<Double> translateY, Supplier<Double> rotate,
            Supplier<Double> howManyBabiesOnBoard) {

        drivetrain = Drivetrain.getInstance();
        this.setName("xDrive");
        this.addRequirements(drivetrain);
        this.translateX = translateX;
        this.translateY = translateY;
        this.rotate = rotate;
        this.howManyBabiesOnBoard = howManyBabiesOnBoard;
    }

    @Override
    public void initialize() {
        drivetrain.postStatus("xDrive");
    }

    @Override
    public void execute() {
        if (Math.abs(translateX.get()) <= Constants.OperatorConstants.Driver.kDeadzone
                && Math.abs(translateY.get()) <= Constants.OperatorConstants.Driver.kDeadzone
                && Math.abs(rotate.get()) <= Constants.OperatorConstants.Driver.kDeadzone) {

            swerveRequest = new SwerveRequest.SwerveDriveBrake();
        } else {
            swerveRequest = new SwerveRequest.FieldCentric()
                    .withVelocityX(-translateX.get()
                            * Constants.SwerveConstants.SwerveSpeeds.kMaxSpeedMetersPerSecond)
                    .withVelocityY(-translateY.get()
                            * Constants.SwerveConstants.SwerveSpeeds.kMaxSpeedMetersPerSecond)
                    .withRotationalRate(-rotate.get()
                            * Constants.SwerveConstants.SwerveSpeeds.kMaxAngularSpeedRadiansPerSecond)
                    .withSlowDown(1 - howManyBabiesOnBoard.get())
                    .withRotationalDeadband(Constants.OperatorConstants.Driver.rotationalDeadband)
                    .withDeadband(Constants.OperatorConstants.Driver.deadband);
        }

        drivetrain.setControl(swerveRequest);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
    }
}
