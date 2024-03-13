package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class oldClimber extends SubsystemBase {
    private TalonFX leftClimber, rightClimber;
    private Solenoid leftClimberBrake, rightClimberBrake;
    public boolean leftSelected = true;

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

    private oldClimber() {
        leftClimber = new TalonFX(Constants.ClimberConstants.leftClimberMotorID, "*");
        rightClimber = new TalonFX(Constants.ClimberConstants.rightClimberMotorID, "*");

        leftClimber.setInverted(true);
        rightClimber.setInverted(true);

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

    public void moveSelected(double value) {
        if (leftSelected) {
            leftClimber.set(value);
        }

        else {
            rightClimber.set(value);
        }
    }

    public void toggleSelected() {
        leftSelected = !leftSelected;
    }

    public boolean getLeftSelected() {
        return leftSelected;
    }

    private static oldClimber mInstance;

    public static oldClimber getInstance() {
        if (mInstance == null) {
            mInstance = new oldClimber();
        }

        return mInstance;
    }
}
