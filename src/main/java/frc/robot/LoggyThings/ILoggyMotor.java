package frc.robot.LoggyThings;

import java.util.EnumSet;
import java.util.HashMap;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DataLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public interface ILoggyMotor {

    /**
     * What type to log, Double, Boolean, or String
     */
    public enum LogItemType {
        DOUBLE,
        BOOLEAN,
        STRING;

        @Override
        public String toString() {
            switch (this) {
                case DOUBLE:
                    return "double";
                case BOOLEAN:
                    return "double";
                case STRING:
                    return "string";
                default:
                    return "double";
            }
        }
    };

    /**
     * Logs data, and provides context if anything has changed
     */
    public class DataLogEntryWithHistory extends DataLogEntry {
        private Object mLastValue;
        private long lastLogTime = 0;

        /**
         * Logs data, and provides context if the value has changed
         * 
         * @param log  the log file currently being used
         * @param name name of the log
         * @param type what data type is being logged {@link LogItemType}
         */
        DataLogEntryWithHistory(DataLog log, String name, LogItemType type) {
            super(log, name, type.toString());
            switch (type) {
                case DOUBLE:
                    mLastValue = Double.valueOf(-99999);
                    break;
                case BOOLEAN:
                    mLastValue = Boolean.valueOf(false);
                    break;
                case STRING:
                    mLastValue = "";
                    break;
                default:
                    mLastValue = Double.valueOf(-99999);
                    break;
            }
        }

        /**
         * Logs a boolean and provides context if the value has changed
         * 
         * @param newValue the boolean to be logged
         * @param now      a long value representing the time
         */
        void logBooleanIfChanged(boolean newValue, long now) {
            if ((((Boolean) mLastValue).booleanValue() != newValue) || (now > (lastLogTime + 500000))) {
                m_log.appendDouble(m_entry, newValue ? 1.0 : 0, now);
                mLastValue = Boolean.valueOf(newValue);
                lastLogTime = now;
            }
        }

        /**
         * Logs a String and provides context if the value has changed
         * 
         * @param newValue the String to be logged
         * @param now      a long value representing the time
         */
        void logStringIfChanged(String newValue, long now) {
            if ((!((String) mLastValue).equals(newValue)) || (now > (lastLogTime + 500000))) {
                m_log.appendString(m_entry, newValue, now);
                mLastValue = newValue;
                lastLogTime = now;
            }
        }

        /**
         * Logs a double and provides context if the value has changed
         * 
         * @param newValue the double to be logged
         * @param now      a long value representing the time
         */
        void logDoubleIfChanged(double newValue, long now) {
            if ((((Double) mLastValue).doubleValue() != newValue) || (now > (lastLogTime + 500000))) {
                m_log.appendDouble(m_entry, newValue, now);
                mLastValue = newValue;
                lastLogTime = now;
            }
        }
    }

    /**
     * Defaults to double
     */
    public enum LogItem {
        // Default logging
        SET_FUNCTION_CONTROL_MODE(LogItemType.STRING),
        SET_FUNCTION_VALUE,
        SET_VOLTAGE,
        SET_SELECTED_SENSOR_POSITION,
        SET_NEUTRAL_MODE_IS_BRAKE(LogItemType.BOOLEAN),

        // CTRE status frame 1
        OUTPUT_PERCENT(LogItemType.DOUBLE),
        FORWARD_LIMIT_SWITCH(LogItemType.BOOLEAN),
        REVERSE_LIMIT_SWITCH(LogItemType.BOOLEAN),
        FORWARD_SOFT_LIMIT,
        REVERSE_SOFT_LIMIT,

        // CTRE status frame 2
        SELECTED_SENSOR_POSITION,
        SELECTED_SENSOR_VELOCITY,

        // CTRE Status frame Brushless Current
        STATOR_CURRENT,
        SUPPLY_CURRENT,

        // CTRE status frame 4
        BUS_VOLTAGE,

        // CTRE status frame 21
        TEMPERATURE,

        HAS_RESET(LogItemType.BOOLEAN),

        // Extras for PID logging

        // CTRE status 13
        CLOSED_LOOP_ERROR,
        // INTEGRAL_ACCUMULATOR,
        // ERROR_DERIVATIVE,
        CLOSED_LOOP_TARGET,

        // Everything only

        // CTRE status frame 1
        OUTPUT_VOLTAGE,

        // CTRE status frame 21
        INTEGRATED_SENSOR_POSITION,
        INTEGRATED_SENSOR_VELOCITY;

        // CONFIG(LogItemType.STRING); // Polled from device, not sent automatically.
        // Check at construction only Not currently implemented

        // Log items in this set are event-driven by set function calls, not periodic
        public static final EnumSet<LogItem> SET_FUNCTION_CALLS = EnumSet.of(SET_FUNCTION_CONTROL_MODE,
                SET_FUNCTION_VALUE, SET_VOLTAGE,
                SET_SELECTED_SENSOR_POSITION, SET_NEUTRAL_MODE_IS_BRAKE);

        public static final EnumSet<LogItem> LOGLEVEL_NONE = EnumSet.noneOf(LogItem.class);
        // public static final EnumSet<LogItem> LOGLEVEL_MINIMAL =
        // EnumSet.of(OUTPUT_PERCENT, FAULTS, BUS_VOLTAGE);
        public static final EnumSet<LogItem> LOGLEVEL_MINIMAL = EnumSet.of(OUTPUT_PERCENT, BUS_VOLTAGE);

        private static final EnumSet<LogItem> DEFAULT_LOG_ADDITIONS = EnumSet.of(SET_FUNCTION_CONTROL_MODE,
                SET_FUNCTION_VALUE, SET_VOLTAGE,
                SET_SELECTED_SENSOR_POSITION, SET_NEUTRAL_MODE_IS_BRAKE, FORWARD_LIMIT_SWITCH,
                REVERSE_LIMIT_SWITCH, FORWARD_SOFT_LIMIT, REVERSE_SOFT_LIMIT, SELECTED_SENSOR_POSITION,
                SELECTED_SENSOR_VELOCITY,
                STATOR_CURRENT, SUPPLY_CURRENT, TEMPERATURE, HAS_RESET);

        public static final EnumSet<LogItem> LOGLEVEL_DEFAULT;

        protected static final EnumSet<LogItem> PID_LOG_ADDITIONS = EnumSet.of(CLOSED_LOOP_ERROR,
                CLOSED_LOOP_TARGET);

        public static final EnumSet<LogItem> LOGLEVEL_PID;

        public static final EnumSet<LogItem> LOGLEVEL_EVERYTHING = EnumSet.allOf(LogItem.class);

        public final LogItemType mLogType;

        static {
            LOGLEVEL_DEFAULT = EnumSet.copyOf(LOGLEVEL_MINIMAL);
            LOGLEVEL_DEFAULT.addAll(DEFAULT_LOG_ADDITIONS);

            LOGLEVEL_PID = EnumSet.copyOf(LOGLEVEL_DEFAULT);
            LOGLEVEL_PID.addAll(PID_LOG_ADDITIONS);
        }

        private LogItem(LogItemType logType) {
            this.mLogType = logType;
        }

        private LogItem() {
            this.mLogType = LogItemType.DOUBLE;
        }

        public LogItemType getLogItemType() {
            return mLogType;
        }
    }

    /**
     * Writes data to the log file constructed
     */
    void writeToLog();

    /**
     * Won't check for new values for more than this period.
     * Defaults to 100ms
     * Logs every 30 seconds when robot is disabled for more than 10 seconds
     * 
     * @param logPeriodSeconds minimum length of seconds to wait before checking for
     *                         new values periodically
     */
    void setMinimumLogPeriod(double logPeriodSeconds);

    /**
     * Defaults to 100ms
     * 
     * @return Minimum length of microseconds to wait before checking for
     *         new values periodically
     */
    long getMinimumLogPeriod();

    /**
     *
     * 
     * @param logLevel a list of {@link LogItem} that will be logged each period
     */
    void setLogLevel_internal(EnumSet<LogItem> logLevel);

    /**
     * 
     * @return the list of {@link LogItem} that will be logged each period
     */
    EnumSet<LogItem> getLogLevel();

    /**
     * Set the level of which to log information
     * 
     * @param logLevel a list of {@link LogItem} that will be logged each period
     */
    default void setLogLevel(EnumSet<LogItem> logLevel) {
        DataLogManager.log("LoggyMotor " + getLogPath() + " log levels set to " + logLevel.toString());
        EnumSet<LogItem> lastLogSet = EnumSet.copyOf(getLogLevel());
        setLogLevel_internal(logLevel);
        EnumSet<LogItem> newLogSet = EnumSet.copyOf(getLogLevel());
        newLogSet.removeAll(lastLogSet);// only new items
        lastLogSet.removeAll(getLogLevel());// only removed items

        for (LogItem item : newLogSet) {// Add new items
            getDataLogEntries().put(item,
                    new DataLogEntryWithHistory(DataLogManager.getLog(), getLogPath() + item.toString(),
                            item.getLogItemType()));
        }

        for (LogItem item : lastLogSet) {// Remove removed items
            getDataLogEntries().remove(item);
        }
    }

    /**
     * 
     * @return the log file path String
     */
    String getLogPath();

    /**
     * 
     * @return a {@link HashMap} of {@link LogItem} and
     *         {@link DataLogEntryWithHistory}
     */
    HashMap<LogItem, DataLogEntryWithHistory> getDataLogEntries();

    /**
     * @return the last time the motor was logged
     */
    long getLastLogTime();
}
