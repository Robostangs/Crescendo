package frc.robot.LoggyThings;

import java.util.EnumSet;
import java.util.HashMap;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.util.WPIUtilJNI;

/**
 * A {@link CANcoder} intizialized using {@link ILoggyMotor}
 */
public class LoggyCANcoder extends CANcoder implements ILoggyMotor {
    
    private EnumSet<ILoggyMotor.LogItem> mLogLevel = EnumSet.noneOf(ILoggyMotor.LogItem.class);
    private HashMap<LogItem, DataLogEntryWithHistory> mDataLogEntries = new HashMap<LogItem, DataLogEntryWithHistory>();
    private long mLogPeriod = 100000;
    private long lastLogTime = (long) Math.abs(Math.random() * 100000);
    private String mLogPath;

    /**
     * Constructs a new LoggyCANcoder and registers it with
     * {@link LoggyThingManager}
     * 
     * @param deviceNumber CAN id
     * @param canbus       canbus path String
     * @param logPath      String path of log file
     * @param logLevel     see {@link ILoggyMotor.LogItem}
     */
    private LoggyCANcoder(int deviceNumber, String canbus, String logPath, EnumSet<ILoggyMotor.LogItem> logLevel) {
        super(deviceNumber, canbus);
        mLogPath = logPath;
        setLogLevel(logLevel);
        LoggyThingManager.getInstance().registerLoggyMotor(this);
    }

    /**
     * Constructs a new LoggyCANcoder and registers it with
     * {@link LoggyThingManager}
     * 
     * @param deviceNumber CAN id
     * @param logPath      String path of log file
     * @param logLevel     see {@link ILoggyMotor.LogItem}
     * @param CANivore     if device is on a CANivore, set to true
     */
    public LoggyCANcoder(int deviceNumber, String logPath, EnumSet<ILoggyMotor.LogItem> logLevel, boolean CANivore) {
        this(deviceNumber, CANivore ? "*" : "", logPath, logLevel);
    }

    /**
     * Constructs a new LoggyCANcoder and registers it with
     * {@link LoggyThingManager}
     * 
     * @param deviceNumber CAN id
     * @param logPath      String path of log file
     * @param CANivore     if device is on a CANivore, set to true
     */
    public LoggyCANcoder(int deviceNumber, String logPath, boolean CANivore) {
        this(deviceNumber, CANivore ? "*" : "", logPath, ILoggyMotor.LogItem.LOGLEVEL_DEFAULT);
    }

    /**
     * Constructs a new LoggyCANcoder and registers it with
     * {@link LoggyThingManager}
     * 
     * @param deviceNumber CAN id
     * @param CANivore     if device is on a CANivore, set to true
     */
    public LoggyCANcoder(int deviceNumber, boolean CANivore) {
        this(deviceNumber, CANivore ? "*" : "", "", ILoggyMotor.LogItem.LOGLEVEL_DEFAULT);
    }

    @Override
    public void writeToLog() {
        lastLogTime = WPIUtilJNI.now();
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
    public String getLogPath() {
        return mLogPath;
    }

    @Override
    public HashMap<LogItem, DataLogEntryWithHistory> getDataLogEntries() {
        return mDataLogEntries;
    }

    @Override
    public long getLastLogTime() {
        return lastLogTime;
    }
}
