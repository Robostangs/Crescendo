package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

//Numbering system for drivetrain: 0 - front right, 1 - front left, 2 - back left, 3 - back right
//0.42545 + 0.254/2

public final class Constants {
	/** Should be 16.542 */
	public static final double fieldLength = Units.inchesToMeters(76.1 + 250.5) * 2;
	public static final double fieldHeight = 8.014;

	public static final double kRange = 20;

	public static final String logDirectory = "";

	public class Vision {
		public static final boolean UseLimelight = true;
		public static final String llAprilTag = "limelight-nick";
		public static final String llAprilTagRear = "limelight-rear";
		public static final String llPython = "limelight-python";

		public static final String llAprilTagRearIP = "http://10.5.48.21:5801/stream.mjpg";
		public static final String llPythonIP = "http://10.5.48.22:5801/stream.mjpg";
		public static final String llAprilTagIP = "http://10.5.48.23:5801/stream.mjpg";
		public static final int llPythonPipelineIndex = 0;
		public static final int llAprilTagPipelineIndex = 1;
		public static final int llAprilTagWithLightsPipelineIndex = 2;

		// This is a publicly available set of confidence values from last year
		// public static final Vector<N3> kPrecisionOfMyVision = VecBuilder.fill(0.1,
		// 0.1, Units.degreesToRadians(10));

		// the lower the number, the more odometry will trust the vision
		public static final Vector<N3> kPrecisionInMyVision = VecBuilder.fill(0.6, 0.6, Units.degreesToRadians(250));
		// public static final double[] SpeakerCoords = { 0.21, 5.55, 1.97 };

		/** This needs to be tuned for the shooter */
		// public static final double[] SpeakerCoords = { 0.45, 5.55, 1.9 };
		// public static final double[] SpeakerCoordsBlue = { 0.3, 5.55, 1.9 };
		// public static final double[] SpeakerCoordsRed = { fieldLength - 0.3, 5.55,
		// 1.9 };
		/** The height (in meters) of the speaker */
		public static final Pose2d SpeakerPoseBlue = new Pose2d(0.15, 5.55, Rotation2d.fromDegrees(0));
		public static final Pose2d SpeakerPoseRed = new Pose2d(fieldLength - SpeakerPoseBlue.getX(),
				SpeakerPoseBlue.getY(), Rotation2d.fromDegrees(180));

		public static final double SpeakerHeight = 1.9;
		public static final Pose3d SpeakerPoseBlue3d = new Pose3d(SpeakerPoseBlue.getX(), SpeakerPoseBlue.getY(),
				SpeakerHeight, new Rotation3d(0, 0, SpeakerPoseBlue.getRotation().getRadians()));
		public static final Pose3d SpeakerPoseRed3d = new Pose3d(SpeakerPoseRed.getX(), SpeakerPoseRed.getY(),
				SpeakerHeight, new Rotation3d(0, 0, SpeakerPoseRed.getRotation().getRadians()));

		/** Highest Y value of the speaker */
		// public static final double SpeakerYUpperBound = 6.12 + 0.5;
		public static final double SpeakerYUpperBound = 6.12;
		/** Lowest Y value of the speaker */
		// public static final double SpeakerYLowerBound = 4.98 - 0.5;
		public static final double SpeakerYLowerBound = 4.98;

		/** Should be 0.57 */
		public static final double SpeakerDeadBand = (SpeakerYUpperBound - SpeakerYLowerBound) / 2;
		// public static final double SpeakerDeadBand = 0.57;
	}

	// TODO: need to redo this for faster swerve
	public class SwerveConstants {
		public enum Target {
			kSpeaker, kAmp, None
		}

		public static final double kMaxSpeedMetersPerSecond = 5;
		public static final double kMaxAngularSpeedMetersPerSecond = 3 * Math.PI;

		/** This is with FOC disabled */
		public static final double maxModuleSpeed = 4.7244;

		/** This is with FOC enabled */
		public static final double maxModuleSpeedFOC = 4.572;

		/** Distance (inches) between the 2 left side CANcoders */
		public static final double driveBaseWidth = Units.inchesToMeters(24.75);
		/** Distance (inches) between the 2 front side CANcoders */
		public static final double driveBaseHeight = Units.inchesToMeters(24.1);

		/**
		 * Picture the front of the robot facing to the right in the XY axis
		 */
		// public static final Translation2d centerOfRotation = new
		// Translation2d(driveBaseWidth / 2, driveBaseHeight / 2);
		public static final Translation2d centerOfRotation = new Translation2d(0, 0);

		/**
		 * distance from the center of the robot to the furthest module (meters) should
		 * be 17.27
		 */
		public static final double driveBaseRadius = Utils.pythagorean(driveBaseWidth / 2, driveBaseHeight / 2);

		public class TunerConstants {
			/** Theoretical free speed (m/s) at 12v applied output; */
			public static final double kSpeedAt12VoltsMetersPerSecond = 4.73;

			// public static final double kMaxSpeedMetersPerSecond =
			// kSpeedAt12VoltsMetersPerSecond;

			// The steer motor uses any SwerveModule.SteerRequestType control request with
			// the output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
			private static final Slot0Configs steerGains = new Slot0Configs().withKP(100).withKI(0).withKD(0.2)
					.withKS(0)
					.withKV(1.5).withKA(0);

			// When using closed-loop control, the drive motor uses the control
			// output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
			private static final Slot0Configs driveGains = new Slot0Configs().withKP(3).withKI(0).withKD(0).withKS(0)
					.withKV(0).withKA(0);

			// private static final Slot0Configs driveGainsVelocity = new Slot0Configs()
			// .withKP(10).withKI(0).withKD(1)
			// .withKS(0).withKV(0).withKA(0);

			// The closed-loop output type to use for the steer motors
			// This affects the PID/FF gains for the steer motors
			private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
			// The closed-loop output type to use for the drive motors;
			// This affects the PID/FF gains for the drive motors
			private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

			/** The stator current at which the wheels start to slip; */
			private static final double kSlipCurrentA = 150.0;

			/**
			 * Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
			 */
			private static final double kCoupleRatio = 3.5714285714285716;

			private static final double kDriveGearRatio = 6.746031746031747;
			private static final double kSteerGearRatio = 21.428571428571427;
			private static final double kWheelRadiusInches = 2;

			private static final boolean kSteerMotorReversed = false;
			private static final boolean kInvertLeftSide = true;
			private static final boolean kInvertRightSide = false;

			// These are only used for simulation
			private static final double kSteerInertia = 0.00001;
			private static final double kDriveInertia = 0.001;
			// Simulated voltage necessary to overcome friction
			private static final double kSteerFrictionVoltage = 0.25;
			private static final double kDriveFrictionVoltage = 0.25;

			private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
					.withDriveMotorGearRatio(kDriveGearRatio).withSteerMotorGearRatio(kSteerGearRatio)
					.withWheelRadius(kWheelRadiusInches).withSlipCurrent(kSlipCurrentA).withSteerMotorGains(steerGains)
					.withDriveMotorGains(driveGains).withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
					.withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
					.withSpeedAt12VoltsMps(kSpeedAt12VoltsMetersPerSecond).withSteerInertia(kSteerInertia)
					.withDriveInertia(kDriveInertia).withSteerFrictionVoltage(kSteerFrictionVoltage)
					.withDriveFrictionVoltage(kDriveFrictionVoltage).withFeedbackSource(SteerFeedbackType.FusedCANcoder)
					.withCouplingGearRatio(kCoupleRatio).withSteerMotorInverted(kSteerMotorReversed);

			private static final String kCANbusName = "*";
			private static final int kPigeonId = 0;
			public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
					.withPigeon2Id(kPigeonId).withCANbusName(kCANbusName);

			// Front Left
			private static final int kFrontLeftDriveMotorId = 11;
			private static final int kFrontLeftSteerMotorId = 12;
			private static final int kFrontLeftEncoderId = 10;
			private static final double kFrontLeftEncoderOffset = -0.320556640625;
			private static final double kFrontLeftXPos = driveBaseWidth / 2;
			private static final double kFrontLeftYPos = driveBaseHeight / 2;

			// Front Right
			private static final int kFrontRightDriveMotorId = 21;
			private static final int kFrontRightSteerMotorId = 22;
			private static final int kFrontRightEncoderId = 20;
			private static final double kFrontRightEncoderOffset = -0.375244140625;
			private static final double kFrontRightXPos = driveBaseWidth / 2;
			private static final double kFrontRightYPos = -driveBaseHeight / 2;

			// Back Left
			private static final int kBackLeftDriveMotorId = 31;
			private static final int kBackLeftSteerMotorId = 32;
			private static final int kBackLeftEncoderId = 30;
			private static final double kBackLeftEncoderOffset = -0.17578125;
			private static final double kBackLeftXPos = -driveBaseWidth / 2;
			private static final double kBackLeftYPos = driveBaseHeight / 2;

			// Back Right
			private static final int kBackRightDriveMotorId = 41;
			private static final int kBackRightSteerMotorId = 42;
			private static final int kBackRightEncoderId = 40;
			private static final double kBackRightEncoderOffset = 0.41455078125;
			private static final double kBackRightXPos = -driveBaseWidth / 2;
			private static final double kBackRightYPos = -driveBaseHeight / 2;

			public static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
					kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId,
					kFrontLeftEncoderOffset, kFrontLeftXPos, kFrontLeftYPos, kInvertLeftSide);
			public static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
					kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId,
					kFrontRightEncoderOffset, kFrontRightXPos, kFrontRightYPos, kInvertRightSide);
			public static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
					kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
					kBackLeftXPos, kBackLeftYPos, kInvertLeftSide);
			public static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
					kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
					kBackRightXPos, kBackRightYPos, kInvertRightSide).withDriveMotorInverted(true);
		}
	}

	public static enum AprilTag {
		NoTag(-1), BlueRightHumanPlayer(1), BlueLeftHumanPlayer(2), RedSpeakerOffset(3), RedSpeaker(4), RedAmp(5),
		BlueAmp(6), BlueSpeaker(7), BlueSpeakerOffset(8), RedRightHumanPlayer(9), RedLeftHumanPlayer(10),
		RedLeftStage(11), RedRightStage(12), RedCenterStage(13), BlueCenterStage(14), BlueLeftStage(15),
		BlueRightStage(16);

		public final int id;

		private AprilTag(int id) {
			this.id = id;
		}

		public static AprilTag fromId(int id) {
			for (AprilTag tag : AprilTag.values()) {
				if (tag.id == id) {
					return tag;
				}
			}
			return NoTag;
		}
	}

	public static final class AutoConstants {

		/* SIM PID */
		// public static final PIDConstants translationPID = new PIDConstants(12.5, 0.5,
		// 0.3);
		// public static final PIDConstants rotationPID = new PIDConstants(1.57, 0.07,
		// 0.9, 1);

		/* IRL PID */
		public static final PIDConstants translationPID = new PIDConstants(0.6, 0.5, 0.4, 5);
		public static final PIDConstants rotationPID = new PIDConstants(1, 0.5, 0.5, 10);

		public static final double kMaxSpeedMetersPerSecond = 4;
		public static final double kMaxAccelerationMetersPerSecondSquared = 4;
		public static final double kMaxAngularSpeedMetersPerSecond = 2.5 * Math.PI;
		public static final double kMaxAngularAccelerationMetersPerSecondSquared = kMaxAngularSpeedMetersPerSecond;

		public static final double kMaxAngularSpeedRadiansPerSecond = Units.degreesToRadians(360d * 2.5);
		public static final double kMaxAngularAccelerationRadiansPerSecondPerSecond = Units.degreesToRadians(360 * 2.5);

		public static final String kFieldObjectName = "path";

		/* NoteAlign constants */
		public static final PIDConstants noteAlignPID = new PIDConstants(0.08, 0.1, 0.01);

		public static final double driveSpeed = 2;

		public static class WayPoints {
			public static class Blue {
				public static final Pose2d kAmp = new Pose2d(1.81, 7.75, Rotation2d.fromDegrees(-90));
				public static final Pose2d kHumanPlayer = new Pose2d(13.8, 1.2, Rotation2d.fromDegrees(0));
				public static final Pose2d kSpeakerLeft = new Pose2d(2.6, 6.45, Rotation2d.fromDegrees(180));
				public static final Pose2d kSpeakerCenter = new Pose2d(2.6, Vision.SpeakerPoseBlue.getY(),
						Rotation2d.fromDegrees(180));
				public static final Pose2d kSpeakerRight = new Pose2d(2.6, 4.65, Rotation2d.fromDegrees(180));

				// these are called "edge positions" in path planner because they are on the
				// edge of the wing line
				public static final Pose2d CenterStartPosition = new Pose2d(1.4, 5.55, Rotation2d.fromDegrees(0));
				public static final Pose2d AmpStartPosition = new Pose2d(0.74, 6.7, Rotation2d.fromDegrees(60));
				public static final Pose2d StageStartPosition = new Pose2d(0.74, 4.41, Rotation2d.fromDegrees(-60));

				public static class Notes {
					public static final Pose2d leftStage = new Pose2d(2.9, 7.0, Rotation2d.fromDegrees(0));
					public static final Pose2d centerStage = new Pose2d(2.9, 5.5, Rotation2d.fromDegrees(0));
					public static final Pose2d rightStage = new Pose2d(2.9, 4.11, Rotation2d.fromDegrees(0));
				}
			}
		}
	}

	public static class MotorConstants {
		public static final double falconFreeSpeedRPM = 6380.0;
		public static final double falconShooterLoadRPM = 6200;
		public static final double falconShooterThresholdRPM = falconFreeSpeedRPM * 0.925;

		/* Kraken x60 Info */
		public static class Kraken {
			public static final double krakenFreeSpeedRotationPerMinute = 5800.0;
			public static final double krakenFreeSpeedRadiansPerSecond = krakenFreeSpeedRotationPerMinute * 2 * Math.PI
					/ 60;
			public static final double krakenStallTorqueNM = 9.37;
			public static final double krakenStallCurrentAmps = 483;
			public static final double krakenPeakPowerWatts = 1405;
			public static final double krakenMaxEfficiencyWattsInOverWattsOut = 0.854;
			public static final double krakenCurrMaxEfficiencyAmps = 37;
		}
	}

	public static class OperatorConstants {
		public static final double kDriverDeadzone = 0.04;

		public static final double kManipDeadzone = 0.07;

		public static final double slowDownMultiplier = 0.5;

		public static final double deadband = SwerveConstants.kMaxSpeedMetersPerSecond * 0.07;
		public static final double rotationalDeadband = SwerveConstants.kMaxAngularSpeedMetersPerSecond * 0.07;

		public static final double setpointTimeout = 2;
	}

	public static class ArmConstants {
		public static final int armMotorID = 53;
		public static final boolean armMotorInverted = true;

		public static final int armEncoderID = 50;

		public static final double kFeedForwardDutyCycle = 0.025;
		public static final double kFeedForwardTorqueCurrent = 6.04;
		public static final double kFeedForwardTorqueCurrentWhileShooting = 8;

		/**
		 * Value that gets multiplied against the FineAdjust input variable, this number
		 * is the max output of the Arm Motor
		 */
		public static final double rateOfMotion = 0.5;

		/** 60 degrees */
		public static final double kArmMaxAngle = 60;
		/** 300 degrees */
		public static final double kArmMinAngle = -67;

		public static final double kArmRangeOfMotion = kArmMaxAngle - kArmMinAngle;

		public static final double shooterOffset = 67.67;

		public static final double hardStopOffset = shooterOffset + kArmMinAngle;

		public static final double kInRangeThreshold = 2.5;

		/*
		 * Interpolation between distance vs shooting angle (horizontal dist. to
		 * speaker, angle to shoot)
		 * Y = a*x^b + c
		 * 
		 * A decrease in C will shoot further up a constant amount, any distance
		 * A decrease in B "bows" the curve in, closer distances will shoot further up
		 * and further values will shoot futher down
		 * A decrease in A shifts curve up, no matter the distance, it will shoot lower,
		 * but closer distances will be effected greater than further distances
		 */
		public static final double a = -1170.66;
		public static final double b = -0.751072;
		public static final double c = 1.98502;

		public static class SetPoints {
			public static final double kSpeaker1 = -45;
			public static final double kSpeaker2 = -40;
			public static final double kSpeaker3 = -30;
			public static final double kSubwoofer = kArmMinAngle;
			public static final double kAmp = 46;
			public static final double kIntake = kArmMinAngle;
			public static final double kHorizontal = 0;
		}
	}

	public static class ShooterConstants {
		public static final int feedMotor = 60;
		public static final int topShooterMotorID = 51;
		public static final int bottomShooterMotorID = 52;
		public static final boolean feedIsInverted = false;
		public static final boolean rightShootIsInverted = true;
		public static final boolean leftShootIsInverted = true;
		public static final boolean intakeIsPositive = true;

		public static final double feederShootValue = 1;
		public static final double feederFeedForward = 0.1;
		public static final double shooterChargeUpTime = 0;

		public static final double feederChargeUpTime = 0;
		public static final double feederReverseFeed = 0;

		public static final double shooterReverseSpeed = -0.1;
		// public static final double feederChargeUpTime = 0.24;
		// public static final double feederChargeUpTime = 0.23;
	}

	public static class IntakeConstants {
		public static final int intakeMotorID = 62;
		public static final int beltMotorID = 61;

		public static final int solenoidID = 0;
		public static final int extraSolenoidID = 4;

		public static final int shooterSensorPWM_ID = 9;
		public static final int beltSensorPWM_ID = 1;

		public static final double kDeployTimeSeconds = 0.25;
		public static final double beltFeedForward = 0.05;

		public static final double beltIntakeSpeed = 0.8;

		public static final boolean intakeMotorInverted = false;
		public static final boolean beltMotorInverted = true;
	}

	public static class ClimberConstants {
        public static class LeftMotor {
            public static final int kId = 54;
            public static final boolean kInverted = true;

            public static final double kGearboxRotationsToMechanismMeters = 1;
            public static final double kMaxExtensionMeters = 100;
            public static final double kExtensionThreshold = 0;
            public static final double kExtensionPower = 0.2;
        }

        public static class RightMotor {
            public static final int kId = 55;
            public static final boolean kInverted = false;

            public static final double kGearboxRotationsToMechanismMeters = LeftMotor.kGearboxRotationsToMechanismMeters;
            public static final double kMaxExtensionMeters = LeftMotor.kMaxExtensionMeters;
            public static final double kExtensionThreshold = LeftMotor.kExtensionThreshold;
            public static final double kExtensionPower = LeftMotor.kExtensionPower;
        }

        public static class LeftBrakeSolenoid {
            public static final int kId = 4;
        }

        public static class RightBrakeSolenoid {
            public static final int kId = 7;
        }

        public static final double kDefaultStatorCurrentLimit = 60;
        public static final double kHomingCurrentLimit = 5;
        public static final double kHomingPower = 0.2;
		public static final double kHardStopPositionRelativeToSwitchMeters = -0.1;
	}

	public static class Lights {
		public static enum LEDState {
			/** No lights (white) */
			kOff(new int[] { 255, 255, 255 }),

			/** Ready To Shoot */
			kGreen(new int[] { 0, 255, 0 }),

			/** Alliance Color */
			kRed(new int[] { 255, 0, 0 }),

			/** Alliance Color */
			kBlue(new int[] { 0, 0, 255 }),

			/** Loaded but not ready to shoot */
			kYellow(new int[] { 255, 255, 0 }),

			/** No Piece in shooter */
			kOrange(new int[] { 255, 165, 0 }),

			/** Low Voltage */
			kPurple(new int[] { 128, 0, 128 }),;

			public final int[] color;

			private LEDState(int[] color) {
				this.color = color;
			}

			/**
			 * Gets the color of the LED state
			 * 
			 * @return will return a 3 element array formatted as [R, G, B]
			 */
			public int[] getColor() {
				return color;
			}
		}

		public static final int CANdleID = 0;
		public static final double lowVoltageThreshold = 12.5;
	}
}
