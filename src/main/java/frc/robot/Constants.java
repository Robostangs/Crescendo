package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

public final class Constants {
    public static final class ShooterConstants{
        // Shooter CAN IDs
        public static final int LEFT_SHOOT_MOTOR_ID = 62; // TODO
        public static final int RIGHT_SHOOT_MOTOR_ID = 61; // TODO
        public static final int FEED_MOTOR_ID = 60; // TODO
        public static final int WRIST_MOTOR_ID = 59; // TODO
        public static final int WRIST_ENCODER_ID = 59; // TODO
        
        // Invert motors
        public static final boolean LEFT_SHOOT_INVERT = false; // TODO
        public static final boolean RIGHT_SHOOT_INVERT = false; // TODO
        public static final boolean FEED_INVERT = false; // TODO
        public static final boolean WRIST_INVERT = false; // TODO

        // Wrist PID
        public static final double WRIST_P = 0; // TODO
        public static final double WRIST_I = 0; // TODO
        public static final double WRIST_D = 0; // TODO

        // Wrist feed forward
        public static final double WRIST_S = 0; // TODO
        public static final double WRIST_G = 0; // TODO
        public static final double WRIST_V = 0; // TODO

        public static final double WRIST_ENCODER_OFFSET = 0; // TODO

        public static final double WRIST_ANGLE_TOLORANCE = Math.toRadians(10); // TODO

        // PID to aim robot towards speaker
        public static final double AIM_P = 0; // TODO
        public static final double AIM_I = 0; // TODO
        public static final double AIM_D = 0; // TODO

        public static final double AIM_ANGLE_TOLORANCE = Math.toRadians(10); // TODO

        // Shooter wheel values TODO: Make this PID later
        public static final double SHOOT_SPEED = 1;
        public static final double SHOOT_SPEED_THRESHOLD = 475; // Max speed is 512
        public static final double FEED_SPEED = 1;
    }

    public static final class VisionConstants {
        public static final boolean USE_LIMELIGHTS_FOR_ODOMETRY = true;
        public static final Vector<N3> VISION_STDS = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(10)); // TODO: these are values that other team used
        public static final double[] SPEAKER_COORDINATES = {0.21, 5.55, 1.97}; // (x, y, z) in meters
    }

    public static final class DrivetrainConstants {
        public static final double MAX_SPEED = TunerConstants.kSpeedAt12VoltsMps;
        public static final double MAX_ANGULAR_RATE = 8 * Math.PI;
    }

    public static final class IntakeConstants {
        // Intake CAN IDs
        public static final int SOLENOID_FWD = 60; // TODO
        public static final int SOLENOID_REV = 60; // TODO
        public static final int INTAKE_MOTOR_ID = 60; // TODO

        public static final boolean INTAKE_MOTOR_REVERSE = false; // TODO

        public static final double INTAKE_SPEED = 1; // TODO

        // NoteAlign constants
        public static final double ALIGN_P = 0.08;
        public static final double ALIGN_I = 0.1;
        public static final double ALIGN_D = 0.01;
        
        public static final double DRIVE_SPEED = 2; // Meters per second (I think)
    }
}
