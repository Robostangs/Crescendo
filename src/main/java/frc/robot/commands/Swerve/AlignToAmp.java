package frc.robot.commands.Swerve;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Lights.LEDState;
import frc.robot.Robot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.subsystems.Drivetrain.SwerveRequest;

public class AlignToAmp extends Command {
    Drivetrain drivetrain;
    Lighting lighting;

    LEDState oldState = LEDState.kOff;
    SwerveRequest.FieldCentricFacingAngle driveRequest;

    Supplier<Double> translateX, translateY, howManyBabiesOnBoard;
    Supplier<Rotation2d> getTargetRotation;

    /**
     * Command to set the drivetrain to brake mode when not moving
     * 
     * @param translateX           the forward to backward movement of the robot
     * @param translateY           the right to left movement of the robot
     * @param howManyBabiesOnBoard 1 - the value of how much to slow down (right
     *                             trigger axis)
     */
    public AlignToAmp(Supplier<Double> translateX, Supplier<Double> translateY, Supplier<Double> howManyBabiesOnBoard) {
        drivetrain = Drivetrain.getInstance();
        lighting = Lighting.getInstance();

        this.addRequirements(drivetrain, lighting);

        if (howManyBabiesOnBoard == null) {
            this.howManyBabiesOnBoard = () -> 0.0;
        }

        else {
            this.howManyBabiesOnBoard = howManyBabiesOnBoard;
        }

        this.translateX = translateX;
        this.translateY = translateY;

        getTargetRotation = () -> {
            return Rotation2d.fromDegrees(-90);
        };
    }

    @Override
    public void initialize() {
        driveRequest = new SwerveRequest.FieldCentricFacingAngle();

        driveRequest.HeadingController = new PhoenixPIDController(12, 12, 1);

        // this is for tuning and now we can tune the PID controller
        SmartDashboard.putData("Align PID", driveRequest.HeadingController);
        drivetrain.postStatus("Aligning");

        driveRequest.Deadband = Constants.OperatorConstants.Driver.deadband;
        driveRequest.RotationalDeadband = Constants.OperatorConstants.Driver.rotationalDeadband * 0.05;
        lighting.autoSetLights(false);
    }

    @Override
    public void execute() {
        lighting.clearAnimations();

        LEDState state;

        if (Intake.getInstance().getShooterSensor()) {

            if (Drivetrain.getInstance().readyToAmp()) {
                if (Arm.getInstance().isInRangeOfTarget(Constants.ArmConstants.SetPoints.kAmp)) {
                    state = LEDState.kGreen;
                }

                else {
                    state = LEDState.kBlue;
                }
            }

            else {
                state = LEDState.kRed;
            }
        }

        else {
            state = LEDState.kRobostangsOrange;
        }

        if (state != oldState) {
            lighting.mCANdle.setLEDs(state.getColor()[0], state.getColor()[1], state.getColor()[2]);
        }

        oldState = state;

        driveRequest.TargetDirection = getTargetRotation.get();

        driveRequest
                .withVelocityX(-translateX.get()
                        * Constants.SwerveConstants.SwerveSpeeds.kMaxSpeedMetersPerSecond)
                .withVelocityY(-translateY.get()
                        * Constants.SwerveConstants.SwerveSpeeds.kMaxSpeedMetersPerSecond)
                .withSlowDown(1 - howManyBabiesOnBoard.get());

        drivetrain.setControl(driveRequest);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
        drivetrain.postStatus("Aligned");
        lighting.autoSetLights(true);
    }

    @Override
    public boolean isFinished() {
        if (Robot.isSimulation()) {
            return false;
        }

        else {
            return !Intake.getInstance().getHolding();
        }

    }
}