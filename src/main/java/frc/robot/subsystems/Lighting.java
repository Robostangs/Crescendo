package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Lights;
import frc.robot.Constants.Lights.LEDState;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain.Drivetrain;

@SuppressWarnings("unused")
public class Lighting extends SubsystemBase {
    CANdle mCANdle;
    LEDState mState;
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

            LEDState state;

            if (DriverStation.isDisabled()) {
                if (Robot.pdh.getVoltage() < Lights.lowVoltageThreshold) {
                    state = LEDState.kWhite;
                }

                else if (DriverStation.getAlliance().isPresent()) {
                    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
                        state = LEDState.kBlue;
                    }

                    else {
                        state = LEDState.kRed;
                    }
                }

                else {
                    state = LEDState.kOff;
                }
            }

            else if (Intake.getInstance().getShooterSensor()) {

                if (Shooter.getInstance().readyToShootAdvanced()) {
                    if (Drivetrain.getInstance().readyToShoot()) {
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

        // // if not in auto light mode
        // else {
        // int[] color = new int[3];

        // color = mState.getColor();

        // if (color != oldColor) {
        // mCANdle.setLEDs(color[0], color[1], color[2]);
        // }

        // oldColor = color;
        // }
    }

    private Lighting() {
        mCANdle = new CANdle(Lights.CANdleID);
        mCANdle.configLEDType(LEDStripType.GRB);

        LEDState state = LEDState.kRobostangsOrange;
        LarsonAnimation animation = new LarsonAnimation(state.getColor()[0], state.getColor()[1], state.getColor()[2]);

        setCANdleLights(state);
        setRightBarAnimation(animation);
        setLeftBarAnimation(animation);
        setLeftClimberSupportAnimation(animation);
        setRightClimberSupportAnimation(animation);
        // mCANdle.animate(new StrobeAnimation(mState.getColor()[0],
        // mState.getColor()[1], mState.getColor()[2], 0, 0.5, Lights.strip1Length, 7));
    }

    private static Lighting mLighting;

    public static Lighting getInstance() {
        if (mLighting == null) {
            mLighting = new Lighting();
        }

        return mLighting;
    }

    public void setLights(LEDState state) {
        autoSetLights(false);
        mState = state;
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

    public static Command getStrobeCommand(LEDState state) {
        int[] color = state.getColor();

        
        return Lighting.getInstance().runOnce(() -> {
            Lighting.getInstance().autoSetLights(false);
            Lighting.getInstance().oldState = LEDState.kBlink;
            Lighting.getInstance().timer.restart();
            Lighting.getInstance().setCANdleLights(state);
            Lighting.getInstance().setRightBarAnimation(new StrobeAnimation(color[0], color[1], color[2]));
            Lighting.getInstance().setLeftBarAnimation(new StrobeAnimation(color[0], color[1], color[2]));
            Lighting.getInstance().setLeftClimberSupportAnimation(new StrobeAnimation(color[0], color[1], color[2]));
            Lighting.getInstance().setRightClimberSupportAnimation(new StrobeAnimation(color[0], color[1], color[2]));
        });
    }

    public boolean getAutoMode() {
        return auto;
    }

    public boolean getTimeExpired() {
        // System.out.println("Timer: " + timer.get());
        return timer.hasElapsed(1);
    }
}