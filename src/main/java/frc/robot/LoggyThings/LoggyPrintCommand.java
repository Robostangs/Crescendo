package frc.robot.LoggyThings;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Appends a value to the constructed log file
 */
public class LoggyPrintCommand extends InstantCommand {
    /**
     * Appends a double to the constructed log file
     * 
     * @param toLog the double to log
     */
    public LoggyPrintCommand(double toLog) {
        super(() -> DataLogManager.log(Double.toString(toLog)));
    }

    /**
     * Appends an integer to the constructed log file
     * 
     * @param toLog the int to log
     */
    public LoggyPrintCommand(int toLog) {
        super(() -> DataLogManager.log(Integer.toString(toLog)));
    }

    /**
     * Appends an Object to the constructed log file
     * 
     * @param toLog the Object to log
     */
    public LoggyPrintCommand(Object toLog) {
        super(() -> DataLogManager.log(toLog.toString()));
    }

    /**
     * Appends a String to the constructed log file
     * 
     * @param toLog the String to log
     */
    public LoggyPrintCommand(String toLog) {
        super(() -> DataLogManager.log(toLog));
    }

    public boolean runsWhenDisabled() {
        return true;
    }
}
