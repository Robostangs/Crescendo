package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

public final class Constants {
    public static final class ShooterConstants{
        // Shooter CAN IDs
        public static final int leftShootMotorID = 62; // TODO
        public static final int rightShootMotorID = 61; // TODO
        public static final int feedMotorID = 60; // TODO
        public static final int wristMotorID = 59; // TODO
        public static final int wristEncoderID = 59; // TODO
        
        // Invert motors
        public static final boolean leftShootInvert = false; // TODO
        public static final boolean rightShootInvert = false; // TODO
        public static final boolean feedInvert = false; // TODO
        public static final boolean wristInvert = false; // TODO

        // Wrist PID
        public static final double wristP = 0; // TODO
        public static final double wristI = 0; // TODO
        public static final double wristD = 0; // TODO

        public static final double wristEncoderOffset = 0; // TODO

        // PID to aim robot towards speaker
        public static final double aimP = 0; // TODO
        public static final double aimI = 0; // TODO
        public static final double aimD = 0; // TODO
    }

    public static final class VisionConstants {
        public static final boolean useLimelightForOdometry = true;
        public static final Vector<N3> visionStds = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(10)); // TODO
        public static final double[] speakerCoordinates = {0.21, 5.55, 1.97}; // (x, y, z) in meters
    }

    public static final class DrivetrainConstants {
        public static final double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
        public static final double MaxAngularRate = 8 * Math.PI;
    }
}
