package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class ShooterConstants{
        // Shooter motor IDs
        public static final int leftShootID = 62; // TODO
        public static final int rightShootID = 61; // TODO
        public static final int feedID = 60; // TODO
        public static final int wristID = 59; // TODO

        // Invert motors
        public static final boolean leftShootInvert = false; // TODO
        public static final boolean rightShootInvert = false; // TODO
        public static final boolean feedInvert = false; // TODO
        public static final boolean wristInvert = false; // TODO

        // Wrist PID
        public static final double wristP = 0;
        public static final double wristI = 0;
        public static final double wristD = 0;
        
        // Vision
        public static final boolean useLimelightForOdometry = true;
        public static final Vector<N3> visionStds = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(10));
    }
}
