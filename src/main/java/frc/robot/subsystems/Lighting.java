package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Lights;
import frc.robot.Constants.Lights.LEDState;
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

        // if not in auto light mode
        else {
            int[] color = new int[3];

            color = mState.getColor();

            if (color != oldColor) {
                mCANdle.setLEDs(color[0], color[1], color[2]);
            }

            oldColor = color;
        }
    }

    private Lighting() {
        mCANdle = new CANdle(Lights.CANdleID);
        mCANdle.configLEDType(LEDStripType.GRB);

        mState = LEDState.kOff;
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
}