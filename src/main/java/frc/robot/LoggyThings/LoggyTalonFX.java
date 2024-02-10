package frc.robot.LoggyThings;

import java.util.EnumSet;
import java.util.HashMap;
import java.util.Optional;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.util.WPIUtilJNI;

/**
 * A {@link TalonFX} intizialized using {@link ILoggyMotor}
 */
public class LoggyTalonFX extends TalonFX implements ILoggyMotor {

    private EnumSet<ILoggyMotor.LogItem> mLogLevel = EnumSet.noneOf(ILoggyMotor.LogItem.class);
    private HashMap<LogItem, DataLogEntryWithHistory> mDataLogEntries = new HashMap<LogItem, DataLogEntryWithHistory>();
    private long mLogPeriod = 100000;
    private long lastLogTime = (long) Math.abs(Math.random() * 100000);
    private String mLogPath;
    private double mForwardSoftLimit, mReverseSoftLimit;

    /**
     * Constructs a new LoggyTalonFX and registers it with
     * {@link LoggyThingManager}
     * 
     * @param deviceNumber CAN id
     * @param canbus       canbus path String
     * @param logPath      String path of log file
     * @param logLevel     see {@link ILoggyMotor.LogItem}
     */
    private LoggyTalonFX(int deviceNumber, String logPath, String canbus, EnumSet<ILoggyMotor.LogItem> logLevel) {
        super(deviceNumber, canbus);
        mLogPath = logPath;
        setLogLevel(logLevel);
        LoggyThingManager.getInstance().registerLoggyMotor(this);
    } /* Does not work with 2 CANivores, only 1 */

    /**
     * Constructs a new LoggyTalonFX and registers it with
     * {@link LoggyThingManager}
     * 
     * @param deviceNumber CAN id
     * @param logPath      String path of log file
     * @param logLevel     see {@link ILoggyMotor.LogItem}
     * @param CANivore     if motor is on a CANivore, set to true
     */
    public LoggyTalonFX(int deviceNumber, String logPath, EnumSet<ILoggyMotor.LogItem> logLevel, boolean CANivore) {

        this(deviceNumber, logPath, CANivore ? "*" : "", logLevel);
    }

    /**
     * Constructs a new LoggyTalonFX and registers it with
     * {@link LoggyThingManager}
     * 
     * @param deviceNumber CAN id
     * @param logPath      String path of log file
     */
    public LoggyTalonFX(int deviceNumber, String logPath, boolean CANivore) {
        this(deviceNumber, logPath, CANivore ? "*" : "", ILoggyMotor.LogItem.LOGLEVEL_DEFAULT);
    }

    /**
     * Constructs a new LoggyTalonFX and registers it with
     * {@link LoggyThingManager}
     * 
     * @param deviceNumber CAN id
     */
    public LoggyTalonFX(int deviceNumber, boolean CANivore) {
        this(
                deviceNumber, "/loggyMotors/" + String.valueOf(deviceNumber) + "/",
                CANivore ? "*" : "",
                ILoggyMotor.LogItem.LOGLEVEL_DEFAULT);
    }

    @Override
    public String getLogPath() {
        return mLogPath;
    }

    @Override
    public void setMinimumLogPeriod(double logPeriodSeconds) {
        mLogPeriod = (long) (logPeriodSeconds * 1e6);
    }

    @Override
    public long getMinimumLogPeriod() {
        return mLogPeriod;
    }

    @Override
    public void setLogLevel_internal(EnumSet<LogItem> logLevel) {
        mLogLevel = logLevel;
    }

    @Override
    public EnumSet<LogItem> getLogLevel() {
        return mLogLevel;
    }

    @Override
    public HashMap<LogItem, DataLogEntryWithHistory> getDataLogEntries() {
        return mDataLogEntries;
    }

    @Override
    public void writeToLog() {
        // Slow down if commanded by manager
        long logPeriod = Long.max(LoggyThingManager.getInstance().getMinGlobalLogPeriod(), mLogPeriod);
        long now = WPIUtilJNI.now();
        if ((now - logPeriod) > lastLogTime) {

            // Only things allowed by the local log level are keys in the datalog entries
            EnumSet<LogItem> potentialLogItems = EnumSet.copyOf(mDataLogEntries.keySet());

            // Only things allowed by the global log level
            potentialLogItems.retainAll(LoggyThingManager.getInstance().getGlobalMaxLogLevel());
            potentialLogItems.removeAll(LogItem.SET_FUNCTION_CALLS);// a set function call, not a periodic status value
            if ((getControlMode().getValue() == ControlModeValue.DutyCycleOut)
                    || (getControlMode().getValue() == ControlModeValue.Follower)
                    || (getControlMode().getValue() == ControlModeValue.MusicTone)
                    || (getControlMode().getValue() == ControlModeValue.TorqueCurrentFOC)) {
                potentialLogItems.removeAll(LogItem.PID_LOG_ADDITIONS);
            }
            for (LogItem thisLogItem : potentialLogItems) {
                DataLogEntryWithHistory thisEntry = mDataLogEntries.get(thisLogItem);
                switch (thisLogItem) {
                    case OUTPUT_PERCENT:
                        thisEntry.logDoubleIfChanged(getDutyCycle().getValue(), now);
                        break;
                    case FORWARD_LIMIT_SWITCH:
                        thisEntry.logBooleanIfChanged(getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround,
                                now);
                        break;
                    case REVERSE_LIMIT_SWITCH:
                        thisEntry.logBooleanIfChanged(getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround,
                                now);
                        break;
                    case FORWARD_SOFT_LIMIT:
                        if (getForwardSoftLimit().isPresent())
                            thisEntry.logDoubleIfChanged(getForwardSoftLimit().get(), now);
                        break;
                    case REVERSE_SOFT_LIMIT:
                        if (getReverseSoftLimit().isPresent())
                            thisEntry.logDoubleIfChanged(getReverseSoftLimit().get(), now);
                        break;
                    case SELECTED_SENSOR_POSITION:
                        thisEntry.logDoubleIfChanged(getPosition().getValue(), now);
                        break;
                    case SELECTED_SENSOR_VELOCITY:
                        thisEntry.logDoubleIfChanged(getVelocity().getValue(), now);
                        break;
                    case STATOR_CURRENT:
                        thisEntry.logDoubleIfChanged(getStatorCurrent().getValue(), now);
                        break;
                    case SUPPLY_CURRENT:
                        thisEntry.logDoubleIfChanged(getSupplyCurrent().getValue(), now);
                        break;
                    case BUS_VOLTAGE:
                        thisEntry.logDoubleIfChanged(getSupplyVoltage().getValue(), now);
                        break;
                    case TEMPERATURE:
                        thisEntry.logDoubleIfChanged(getDeviceTemp().getValue(), now);
                        break;
                    case HAS_RESET:
                        thisEntry.logBooleanIfChanged(hasResetOccurred(), now);
                        break;
                    case CLOSED_LOOP_ERROR:
                        thisEntry.logDoubleIfChanged(getClosedLoopError().getValue(), now);
                        break;
                    case CLOSED_LOOP_TARGET:
                        thisEntry.logDoubleIfChanged(getClosedLoopReference().getValue(), now);
                        break;
                    case OUTPUT_VOLTAGE:
                        thisEntry.logDoubleIfChanged(getMotorVoltage().getValue(), now);
                        break;
                    case INTEGRATED_SENSOR_POSITION:
                        thisEntry.logDoubleIfChanged(getRotorPosition().getValue(), now);
                        break;
                    case INTEGRATED_SENSOR_VELOCITY:
                        thisEntry.logDoubleIfChanged(getRotorVelocity().getValue(), now);
                        break;
                    default:
                        break;
                }
            }
            lastLogTime = WPIUtilJNI.now();
        }
    }

    boolean justFailed = false;

    /**
     * PercentOut replacement.
     * Just enter a speed (double) and the motor will use
     * {@link ControlModeValue#DutyCycleOut} with FOC enabled
     */
    @Override
    public void set(double speed) {
        /* Set speed using DutyCycleOut */
        super.set(speed);

        try {// Don't jeopardize robot functionality
             // Filter the 4 potential log items down to the ones allowed here
            EnumSet<LogItem> potentialLogItems = EnumSet.of(LogItem.SET_FUNCTION_CONTROL_MODE,
                    LogItem.SET_FUNCTION_VALUE);
            potentialLogItems.retainAll(mDataLogEntries.keySet());
            potentialLogItems.retainAll(LoggyThingManager.getInstance().getGlobalMaxLogLevel());
            long now = WPIUtilJNI.now();
            for (LogItem thisLogItem : potentialLogItems) {
                DataLogEntryWithHistory thisEntry = mDataLogEntries.get(thisLogItem);

                switch (thisLogItem) {
                    case SET_FUNCTION_CONTROL_MODE:
                        thisEntry.logStringIfChanged(getControlMode().toString(), now);
                        break;
                    case SET_FUNCTION_VALUE:
                        thisEntry.logDoubleIfChanged(speed, now);
                        break;
                    default:
                        break;
                }
            }
            justFailed = false;
        } catch (Exception e) {
            if (!justFailed) {
                e.printStackTrace();
                justFailed = true;
            }
        }
    }

    @Override
    public StatusCode setControl(ControlRequest request) {
        if (super.setControl(request) != StatusCode.OK) {
            return StatusCode.GeneralError;
        }

        try {
            EnumSet<LogItem> potentialLogItems = EnumSet.of(LogItem.SET_FUNCTION_CONTROL_MODE);
            potentialLogItems.retainAll(mDataLogEntries.keySet());
            potentialLogItems.retainAll(LoggyThingManager.getInstance().getGlobalMaxLogLevel());
            long now = WPIUtilJNI.now();
            for (LogItem thisLogItem : potentialLogItems) {
                DataLogEntryWithHistory thisEntry = mDataLogEntries.get(thisLogItem);

                switch (thisLogItem) {
                    case SET_FUNCTION_CONTROL_MODE:
                        thisEntry.logStringIfChanged(request.getName(), now);
                        break;
                    default:
                        break;
                }
            }
            justFailed = false;
        } catch (Exception e) {
            if (!justFailed) {
                e.printStackTrace();
                justFailed = true;
                return StatusCode.GeneralError;
            }
        }
        return StatusCode.OK;
    }

    // @Override
    // public StatusCode

    /**
     * configre TalonFX slot, 🚨 do not pass in {@link Slot1Configs} for swerve motors 🚨
     * 
     * @param config you can pass in a {@link Slot0Configs}, {@link Slot1Configs},
     *               {@link Slot2Configs}, or {@link TalonFXConfiguration
     * @return {@link StatusCode#OK} if successful, else
     *         {@link StatusCode#ConfigFailed}
     */
    public StatusCode setSlotConfig(Object config) {
        if (config instanceof Slot0Configs) {
            return super.getConfigurator().apply((Slot0Configs) config);
        } else if (config instanceof Slot1Configs) {
            return super.getConfigurator().apply((Slot1Configs) config);
        } else if (config instanceof Slot2Configs) {
            return super.getConfigurator().apply((Slot2Configs) config);
        } else if (config instanceof TalonFXConfiguration) {
            return super.getConfigurator().apply((TalonFXConfiguration) config);
        } else {
            return StatusCode.ConfigFailed;
        }
    }

    @Override
    public void setVoltage(double outputVolts) {
        super.setVoltage(outputVolts);
        try {
            if (mDataLogEntries.keySet().contains(LogItem.SET_VOLTAGE)
                    && LoggyThingManager.getInstance().getGlobalMaxLogLevel().contains(LogItem.SET_VOLTAGE)) {
                mDataLogEntries.get(LogItem.SET_VOLTAGE).logDoubleIfChanged(outputVolts, WPIUtilJNI.now());
                justFailed = false;
            }
        } catch (Exception e) {
            if (!justFailed) {// don;t spam log
                e.printStackTrace();
                justFailed = true;
            }
        }
    }

    @Override
    public StatusCode setPosition(double sensorPos) {
        return setPosition(sensorPos, 0);
    }

    @Override
    public StatusCode setPosition(double newValue, double timeoutSeconds) {
        try {
            if (mDataLogEntries.keySet().contains(LogItem.SET_SELECTED_SENSOR_POSITION)
                    && LoggyThingManager.getInstance().getGlobalMaxLogLevel()
                            .contains(LogItem.SET_SELECTED_SENSOR_POSITION)) {
                mDataLogEntries.get(LogItem.SET_SELECTED_SENSOR_POSITION).logDoubleIfChanged(newValue,
                        WPIUtilJNI.now());
                justFailed = false;
            }
        } catch (Exception e) {
            if (!justFailed) {
                e.printStackTrace();
                justFailed = true;
            }
        }
        return super.setPosition(newValue, timeoutSeconds);
    }

    @Override
    public void setNeutralMode(NeutralModeValue neutralMode) {
        try {
            if (mDataLogEntries.keySet().contains(LogItem.SET_NEUTRAL_MODE_IS_BRAKE)
                    && LoggyThingManager.getInstance().getGlobalMaxLogLevel()
                            .contains(LogItem.SET_NEUTRAL_MODE_IS_BRAKE)) {
                mDataLogEntries.get(LogItem.SET_NEUTRAL_MODE_IS_BRAKE)
                        .logBooleanIfChanged(neutralMode == NeutralModeValue.Brake, WPIUtilJNI.now());
                justFailed = false;
            }
        } catch (Exception e) {
            if (!justFailed) {
                e.printStackTrace();
                justFailed = true;
            }
        }
        super.setNeutralMode(neutralMode);
    }

    @Override
    public long getLastLogTime() {
        return lastLogTime;
    }

    /**
     * set software limitations on this motor
     * 
     * @param configs the config class to be applied to this motor
     */
    public void setSoftLimit(HardwareLimitSwitchConfigs configs) {
        mForwardSoftLimit = configs.ForwardLimitAutosetPositionValue;
        mReverseSoftLimit = configs.ReverseLimitAutosetPositionValue;
        super.getConfigurator().apply(configs);
    }

    public Optional<Double> getForwardSoftLimit() {
        return Optional.ofNullable(mForwardSoftLimit);
    }

    public Optional<Double> getReverseSoftLimit() {
        return Optional.ofNullable(mReverseSoftLimit);
    }
}