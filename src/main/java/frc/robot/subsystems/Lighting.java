package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Lights;
import frc.robot.Constants.Lights.LEDState;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain.Drivetrain;

import java.util.function.Supplier;

public class Lighting extends SubsystemBase {
    CANdle mCANdle;
    boolean auto = false;
    int[] oldColor = new int[3];
    LEDState oldState;

    boolean blink = false;
    Timer timer = new Timer();

    @Override
    public void periodic() {
        if (auto) {
            mCANdle.clearAnimation(0);
            mCANdle.clearAnimation(1);
            mCANdle.clearAnimation(2);
            mCANdle.clearAnimation(3);
            mCANdle.clearAnimation(4);

            LEDState state;

            if (DriverStation.isDisabled()) {
                if (Robot.pdh.getVoltage() < Lights.lowVoltageThreshold) {
                    state = LEDState.kWhite;
                }

                else {
                    state = LEDState.kOff;
                }
            }

            else if (Intake.getInstance().getShooterSensor()) {

                if (Drivetrain.getInstance().readyToShoot()) {
                    if (Shooter.getInstance().readyToShootAdvanced()) {
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
                mCANdle.setLEDs(state.getColor()[0], state.getColor()[1], state.getColor()[2]);
            }

            oldState = state;
        }

        else {
            if (timer.advanceIfElapsed(3) && DriverStation.isEnabled()) {
                mLighting.autoSetLights(true);
            }
        }
    }

    private Lighting() {
        mCANdle = new CANdle(Lights.CANdleID);
        mCANdle.configLEDType(LEDStripType.GRB);
    }

    private static Lighting mLighting;

    public static Lighting getInstance() {
        if (mLighting == null) {
            mLighting = new Lighting();
        }

        return mLighting;
    }

    public void autoSetLights(boolean autoSet) {
        auto = autoSet;
    }

    public void setRightBarColor(int r, int g, int b) {
        mCANdle.setLEDs(r, g, b, 0, 7, Lights.strip1Length);
    }

    public void setLeftBarColor(int r, int g, int b) {
        mCANdle.setLEDs(r, g, b, 0, 7 + Lights.strip1Length, Lights.strip2Length);
    }

    public void setLeftClimberSupport(int r, int g, int b) {
        mCANdle.setLEDs(r, g, b, 0, 7 + Lights.strip1Length + Lights.strip2Length, Lights.strip3Length);
    }

    public void setRightClimberSupport(int r, int g, int b) {
        mCANdle.setLEDs(r, g, b, 0, 7 + Lights.strip1Length + Lights.strip2Length + Lights.strip3Length,
                Lights.strip4Length);
    }

    public void setRightBarAnimation(Animation animation) {
        animation.setSpeed(Lights.animationSpeed);
        animation.setLedOffset(8);
        animation.setNumLed(Lights.strip1Length);
        mCANdle.animate(animation, 0);
    }

    public void setLeftBarAnimation(Animation animation) {
        animation.setSpeed(Lights.animationSpeed);
        animation.setLedOffset(8 + Lights.strip1Length);
        animation.setNumLed(Lights.strip2Length);
        mCANdle.animate(animation, 1);
    }

    public void setLeftClimberSupportAnimation(Animation animation) {
        animation.setSpeed(Lights.animationSpeed);
        animation.setLedOffset(8 + Lights.strip1Length + Lights.strip2Length);
        animation.setNumLed(Lights.strip3Length);
        mCANdle.animate(animation, 2);
    }

    public void setRightClimberSupportAnimation(Animation animation) {
        animation.setSpeed(Lights.animationSpeed);
        animation.setLedOffset(8 + Lights.strip1Length + Lights.strip2Length + Lights.strip3Length);
        animation.setNumLed(Lights.strip4Length);
        mCANdle.animate(animation, 3);
    }

    public void setCANdleLights(LEDState state) {
        mCANdle.setLEDs(state.getColor()[0], state.getColor()[1], state.getColor()[2], 0, 0, 8);
    }

    public void setCANdleAnimation(Animation animation) {
        animation.setSpeed(Lights.animationSpeed);
        animation.setLedOffset(0);
        animation.setNumLed(8);
        mCANdle.animate(animation, 4);
    }

    public static Command getStrobeCommand(Supplier<LEDState> state) {
        // int[] color = state.get().getColor();

        return Lighting.getInstance().runOnce(() -> {
            int[] color = state.get().getColor();
            Lighting.getInstance().autoSetLights(false);
            Lighting.getInstance().oldState = LEDState.kCustom;
            Lighting.getInstance().timer.restart();
            Lighting.getInstance().setCANdleAnimation(new StrobeAnimation(color[0], color[1], color[2]));
            Lighting.getInstance().setRightBarAnimation(new StrobeAnimation(color[0], color[1], color[2]));
            Lighting.getInstance().setLeftBarAnimation(new StrobeAnimation(color[0], color[1], color[2]));
            Lighting.getInstance().setLeftClimberSupportAnimation(new StrobeAnimation(color[0], color[1], color[2]));
            Lighting.getInstance().setRightClimberSupportAnimation(new StrobeAnimation(color[0], color[1], color[2]));
        }).ignoringDisable(true);
    }

    public static Command getLarsonCommand(LEDState state) {
        int[] color = state.getColor();

        return Lighting.getInstance().runOnce(() -> {
            Lighting.getInstance().autoSetLights(false);
            Lighting.getInstance().oldState = LEDState.kCustom;
            Lighting.getInstance().timer.restart();
            Lighting.getInstance().setCANdleLights(state);
            Lighting.getInstance().setRightBarAnimation(new LarsonAnimation(color[0], color[1], color[2]));
            Lighting.getInstance().setLeftBarAnimation(new LarsonAnimation(color[0], color[1], color[2]));
            Lighting.getInstance().setLeftClimberSupportAnimation(new LarsonAnimation(color[0], color[1], color[2]));
            Lighting.getInstance().setRightClimberSupportAnimation(new LarsonAnimation(color[0], color[1], color[2]));
        }).ignoringDisable(true);
    }
}