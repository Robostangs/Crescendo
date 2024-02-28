package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LoggyThings.LoggyTalonFX;

public class Climber extends SubsystemBase {
    private LoggyTalonFX leftClimber, rightClimber;
    private Solenoid leftClimberBrake, rightClimberBrake;

    @Override
    public void periodic() {
        if (leftClimber.get() == 0d) {
            leftClimberBrake.set(false);
        } else {
            leftClimberBrake.set(true);
        }

        if (rightClimber.get() == 0d) {
            rightClimberBrake.set(false);
        } else {
            rightClimberBrake.set(true);
        }
    }

    private Climber() {
        leftClimber = new LoggyTalonFX(Constants.ClimberConstants.leftClimberMotorID, true);
        rightClimber = new LoggyTalonFX(Constants.ClimberConstants.rightClimberMotorID, true);

        leftClimber.setInverted(true);
        rightClimber.setInverted(false);

        leftClimberBrake = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.ClimberConstants.leftClimberBrakeID);
        rightClimberBrake = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.ClimberConstants.rightClimberBrakeID);
    }

    public void setSpeed(double speed) {
        // if (!(leftClimber.getPosition().getValueAsDouble() >= Constants.ClimberConstants.climberMaxExtensionRotations
        //         && speed > 0
        //         || leftClimber.getPosition().getValueAsDouble() <= Constants.ClimberConstants.climberMinExtensionRotations
        //                 && speed < 0)) {
        //     leftClimber.set(speed);
        // }

        // if (!(rightClimber.getPosition().getValueAsDouble() >= Constants.ClimberConstants.climberMaxExtensionRotations
        //         && speed > 0
        //         || rightClimber.getPosition().getValueAsDouble() <= Constants.ClimberConstants.climberMinExtensionRotations
        //                 && speed < 0)) {
        //     rightClimber.set(speed);
        // }

        leftClimber.set(speed);
        rightClimber.set(speed);
    }

    public void goUp() {
        setSpeed(Constants.ClimberConstants.climberSpeed);
    }

    public void goDown() {
        setSpeed(-Constants.ClimberConstants.climberSpeed);
    }

    public void stop() {
        setSpeed(0);
    }

    private static Climber mInstance;

    public static Climber getInstance() {
        if (mInstance == null) {
            mInstance = new Climber();
        }

        return mInstance;
    }
}
