package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

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
    
    public static Runnable startTimer = () -> {
        Lighting.getInstance().timer.start();
    };

    @Override
    public void periodic() {
        if (auto) {
            timer.reset();

            clearAnimations();

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
            if (timer.get() > 3 && DriverStation.isEnabled()) {
                mLighting.autoSetLights(true);
            }
        }
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
        animation.setLedOffset(8);
        animation.setNumLed(Lights.strip1Length);
        mCANdle.animate(animation, 0);
    }

    public void setLeftBarAnimation(Animation animation) {
        animation.setLedOffset(8 + Lights.strip1Length);
        animation.setNumLed(Lights.strip2Length);
        mCANdle.animate(animation, 1);
    }

    public void setLeftClimberSupportAnimation(Animation animation) {
        animation.setLedOffset(8 + Lights.strip1Length + Lights.strip2Length);
        animation.setNumLed(Lights.strip3Length);
        mCANdle.animate(animation, 2);
    }

    public void setRightClimberSupportAnimation(Animation animation) {
        animation.setLedOffset(8 + Lights.strip1Length + Lights.strip2Length + Lights.strip3Length);
        animation.setNumLed(Lights.strip4Length);
        mCANdle.animate(animation, 3);
    }

    public void setCANdleLights(LEDState state) {
        mCANdle.setLEDs(state.getColor()[0], state.getColor()[1], state.getColor()[2], 0, 0, 8);
    }

    public void setCANdleAnimation(Animation animation) {
        animation.setLedOffset(0);
        animation.setNumLed(8);
        mCANdle.animate(animation, 4);
    }

    public static Command getStrobeCommand(Supplier<LEDState> state) {
        return Lighting.getInstance().runOnce(() -> {
            int[] color = state.get().getColor();
            StrobeAnimation animation = new StrobeAnimation(color[0], color[1], color[2]);
            animation.setSpeed(Lights.strobeAnimationSpeed);
            Lighting.getInstance().autoSetLights(false);
            Lighting.getInstance().oldState = LEDState.kCustom;
            Lighting.getInstance().timer.reset();
            Lighting.getInstance().setCANdleAnimation(animation);
            Lighting.getInstance().setRightBarAnimation(animation);
            Lighting.getInstance().setLeftBarAnimation(animation);
            Lighting.getInstance().setLeftClimberSupportAnimation(animation);
            Lighting.getInstance().setRightClimberSupportAnimation(animation);
        }).ignoringDisable(true);
    }

    public static Command getLarsonCommand(Supplier<LEDState> state) {
        return Lighting.getInstance().runOnce(() -> {
            int[] color = state.get().getColor();
            LarsonAnimation animation = new LarsonAnimation(color[0], color[1], color[2]);
            animation.setSize(Lights.larsonAnimationSize);
            animation.setSpeed(Lights.larsonAnimationSpeed);
            animation.setBounceMode(BounceMode.Front);
            Lighting.getInstance().autoSetLights(false);
            Lighting.getInstance().oldState = LEDState.kCustom;
            Lighting.getInstance().timer.reset();
            Lighting.getInstance().setCANdleLights(state.get());
            Lighting.getInstance().setRightBarAnimation(animation);
            Lighting.getInstance().setLeftBarAnimation(animation);
            Lighting.getInstance().setLeftClimberSupportAnimation(animation);
            Lighting.getInstance().setRightClimberSupportAnimation(animation);
        }).ignoringDisable(true);
    }

    public void clearAnimations() {
        mCANdle.clearAnimation(0);
        mCANdle.clearAnimation(1);
        mCANdle.clearAnimation(2);
        mCANdle.clearAnimation(3);
        mCANdle.clearAnimation(4);
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
}