package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public abstract class Constants {
    public static abstract class Vision {
        public static final boolean useLimelightForOdometry = true;
        public static final Vector<N3> visionStds = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(10));
    }
}
