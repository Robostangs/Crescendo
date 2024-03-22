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
		public static final String llAprilTag = "limelight-front";
		public static final String llAprilTagRear = "limelight-rear";
		public static final String llPython = "limelight-python";

		public static final String llAprilTagRearIP = "http://10.5.48.21:5800/stream.mjpg";
		public static final String llPythonIP = "http://10.5.48.22:5800/stream.mjpg";
		public static final String llAprilTagIP = "http://10.5.48.23:5800/stream.mjpg";
		public static final int llPythonPipelineIndex = 0;
		public static final int llAprilTagPipelineIndex = 1;
		public static final int llAprilTagWithLightsPipelineIndex = 2;

		// the lower the number, the more odometry will trust the vision
		public static final Vector<N3> kPrecisionInMyVision = VecBuilder.fill(0.42, 0.42, Units.degreesToRadians(100));

		/** The height (in meters) of the speaker */
		public static final Pose2d SpeakerPoseBlue = new Pose2d(0, 5.55, Rotation2d.fromDegrees(0));
		public static final Pose2d SpeakerPoseRed = new Pose2d(fieldLength - SpeakerPoseBlue.getX(),
				SpeakerPoseBlue.getY(), Rotation2d.fromDegrees(180));

		public static final double SpeakerHeightMeters = 2.032;

		public static final Pose3d SpeakerPoseBlue3d = new Pose3d(SpeakerPoseBlue.getX(), SpeakerPoseBlue.getY(),
				SpeakerHeightMeters, new Rotation3d(0, 0, SpeakerPoseBlue.getRotation().getRadians()));
		public static final Pose3d SpeakerPoseRed3d = new Pose3d(SpeakerPoseRed.getX(), SpeakerPoseRed.getY(),
				SpeakerHeightMeters, new Rotation3d(0, 0, SpeakerPoseRed.getRotation().getRadians()));
	}

	public class SwerveConstants {
		public enum Target {
			kSpeaker, kAmp, None
		}

		public static final double kMaxSpeedMetersPerSecond = 6;
		public static final double kMaxAngularSpeedMetersPerSecond = 4 * Math.PI;

		public static final double maxModuleSpeed = 6;

		/** Distance (inches) between the 2 left side CANcoders */
		public static final double driveBaseWidth = 24.75;
		/** Distance (inches) between the 2 front side CANcoders */
		public static final double driveBaseHeight = 24.1;

		/**
		 * Picture the front of the robot facing to the right in the XY axis
		 */
		public static final Translation2d centerOfRotation = new Translation2d(0, 0);

		/**
		 * distance from the center of the robot to the furthest module (meters) should
		 * be 17.27
		 */
		public static final double driveBaseRadius = Units
				.inchesToMeters(Utils.pythagorean(driveBaseWidth / 2, driveBaseHeight / 2));

		public class TunerConstants {
			// The steer motor uses any SwerveModule.SteerRequestType control request with
			// the
			// output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
			private static final Slot0Configs steerGains = new Slot0Configs()
					.withKP(100).withKI(0).withKD(0.2)
					.withKS(0).withKV(1.5).withKA(0);

			// When using closed-loop control, the drive motor uses the control
			// output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
			private static final Slot0Configs driveGains = new Slot0Configs()
					.withKP(3).withKI(0).withKD(0)
					.withKS(0).withKV(0).withKA(0);

			/** The closed-loop output type to use for the steer motors */
			private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;

			/** The closed-loop output type to use for the drive motors */
			private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

			/** The stator current at which the wheels start to slip */
			private static final double kSlipCurrentA = 80.0;

			/** Theoretical free speed (m/s) at 12v applied output */
			public static final double kSpeedAt12VoltsMps = 5.96;

			// TODO: tune this number
			/**
			 * Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns
			 * This may need to be tuned to your individual robot
			 */
			// private static final double kCoupleRatio = 3.125;
			private static final double kCoupleRatio = 6.12;

			private static final double kDriveGearRatio = 5.357142857142857;
			private static final double kSteerGearRatio = 21.428571428571427;
			private static final double kWheelRadiusInches = 2 * 6.3 / 6.6;

			private static final boolean kSteerMotorReversed = false;
			private static final boolean kInvertLeftSide = true;
			private static final boolean kInvertRightSide = false;

			private static final String kCANbusName = "*";
			private static final int kPigeonId = 0;

			/** These are only used for simulation */
			private static final double kSteerInertia = 0.00001, kDriveInertia = 0.001;

			/** Simulated voltage necessary to overcome friction */
			private static final double kSteerFrictionVoltage = 0.25, kDriveFrictionVoltage = 0.25;

			public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
					.withPigeon2Id(kPigeonId)
					.withCANbusName(kCANbusName);

			private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
					.withDriveMotorGearRatio(kDriveGearRatio)
					.withSteerMotorGearRatio(kSteerGearRatio)
					.withWheelRadius(kWheelRadiusInches)
					.withSlipCurrent(kSlipCurrentA)
					.withSteerMotorGains(steerGains)
					.withDriveMotorGains(driveGains)
					.withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
					.withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
					.withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
					.withSteerInertia(kSteerInertia)
					.withDriveInertia(kDriveInertia)
					.withSteerFrictionVoltage(kSteerFrictionVoltage)
					.withDriveFrictionVoltage(kDriveFrictionVoltage)
					.withFeedbackSource(SteerFeedbackType.FusedCANcoder)
					.withCouplingGearRatio(kCoupleRatio)
					.withSteerMotorInverted(kSteerMotorReversed);

			// Front Left
			private static final int kFrontLeftDriveMotorId = 11;
			private static final int kFrontLeftSteerMotorId = 12;
			private static final int kFrontLeftEncoderId = 10;
			private static final double kFrontLeftEncoderOffset = -0.30908203125;

			private static final double kFrontLeftXPosInches = 12.875;
			private static final double kFrontLeftYPosInches = 12.05;

			// Front Right
			private static final int kFrontRightDriveMotorId = 21;
			private static final int kFrontRightSteerMotorId = 22;
			private static final int kFrontRightEncoderId = 20;
			private static final double kFrontRightEncoderOffset = -0.370361328125;

			private static final double kFrontRightXPosInches = 12.875;
			private static final double kFrontRightYPosInches = -12.05;

			// Back Left
			private static final int kBackLeftDriveMotorId = 31;
			private static final int kBackLeftSteerMotorId = 32;
			private static final int kBackLeftEncoderId = 30;
			private static final double kBackLeftEncoderOffset = -0.18701171875;

			private static final double kBackLeftXPosInches = -12.875;
			private static final double kBackLeftYPosInches = 12.05;

			// Back Right
			private static final int kBackRightDriveMotorId = 41;
			private static final int kBackRightSteerMotorId = 42;
			private static final int kBackRightEncoderId = 40;
			private static final double kBackRightEncoderOffset = -0.075439453125;

			private static final double kBackRightXPosInches = -12.875;
			private static final double kBackRightYPosInches = -12.05;

			public static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
					kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
					Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches),
					kInvertLeftSide);
			public static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
					kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset,
					Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches),
					kInvertRightSide);
			public static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
					kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
					Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches),
					kInvertLeftSide);
			public static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
					kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
					Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches),
					kInvertRightSide);
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
		// public static final PIDConstants translationPID = new
		// PIDConstants(12.5, 0.5, 0.3);
		// public static final PIDConstants rotationPID = new
		// PIDConstants(1.57, 0.07, 0.9, 1);

		/* Old PID */
		// public static final PIDConstants translationPID = new PIDConstants(0.6, 0.1,
		// 0.4, 5);
		// public static final PIDConstants rotationPID = new PIDConstants(1, 0.5, 0.5,
		// 10);

		public static final PIDConstants translationPID = new PIDConstants(0.85, 0.05, 0.1, 0.5);
		public static final PIDConstants rotationPID = new PIDConstants(4, 0, 0, 1);

		public static final double kMaxSpeedMetersPerSecond = SwerveConstants.kMaxSpeedMetersPerSecond;
		public static final double kMaxAccelerationMetersPerSecondSquared = 4;
		public static final double kMaxAngularSpeedMetersPerSecond = SwerveConstants.kMaxAngularSpeedMetersPerSecond;
		public static final double kMaxAngularAccelerationMetersPerSecondSquared = kMaxAngularSpeedMetersPerSecond * 4
				/ 6;

		public static final double kMaxAngularSpeedRadiansPerSecond = Units.degreesToRadians(360d * 4);
		public static final double kMaxAngularAccelerationRadiansPerSecondPerSecond = Units.degreesToRadians(360 * 2.5);

		public static final String kFieldObjectName = "path";

		public static class WayPoints {
			public static class Blue {
				public static final Pose2d kAmp = new Pose2d(1.81, 7.75, Rotation2d.fromDegrees(-90));
				public static final Pose2d kHumanPlayer = new Pose2d(13.8, 1.2, Rotation2d.fromDegrees(0));
				public static final Pose2d kSpeakerLeft = new Pose2d(2.6, 6.45, Rotation2d.fromDegrees(180));
				public static final Pose2d kSpeakerCenter = new Pose2d(2.6, Vision.SpeakerPoseBlue.getY(),
						Rotation2d.fromDegrees(180));
				public static final Pose2d kSpeakerRight = new Pose2d(2.6, 4.65, Rotation2d.fromDegrees(180));

				public static final Pose2d CenterStartPosition = new Pose2d(1.4, 5.55, Rotation2d.fromDegrees(0));
				public static final Pose2d AmpStartPosition = new Pose2d(0.74, 6.7, Rotation2d.fromDegrees(60));
				public static final Pose2d StageStartPosition = new Pose2d(0.74, 4.41, Rotation2d.fromDegrees(-60));

				public static class StartingNotes {
					public static final Pose2d amp = new Pose2d(2.9, 7.0, Rotation2d.fromDegrees(0));
					public static final Pose2d center = new Pose2d(2.9, 5.5, Rotation2d.fromDegrees(0));
					public static final Pose2d stage = new Pose2d(2.9, 4.11, Rotation2d.fromDegrees(0));
				}
			}
		}
	}

	public static class MotorConstants {
		public static final double falconFreeSpeedRPM = 6380.0;
		public static final double falconShooterLoadRPM = 5400;
		public static final double falconShooterThresholdRPM = falconShooterLoadRPM * 0.9;

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
		public static final int armCoderID = 50;
		public static final boolean armMotorInverted = true;

		// TODO: need new values for new shooter, will probably be lower
		public static final double kFeedForwardDutyCycle = 0.025;
		public static final double kFeedForwardTorqueCurrent = 6.04 - 1;
		public static final double kFeedForwardTorqueCurrentWhileShooting = 8 - 1;

		public static final double shooterTrapezoidalOffset = 2.6;
		public static final double angleOfShooterReferencePointSittingOnHardStop = 60.4;

		/**
		 * Value that gets multiplied against the FineAdjust input variable, this number
		 * is the max output of the Arm Motor
		 */
		public static final double rateOfMotion = 0.5;


		public static final double kArmMaxAngle = 60;
		public static final double kArmMinAngle = -61.6 - shooterTrapezoidalOffset;
		public static final double kArmRangeOfMotion = kArmMaxAngle - kArmMinAngle;
		public static final double shooterOffset = 67.3 + shooterTrapezoidalOffset;
		public static final double kInRangeThreshold = 3.25;

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
			public static final double kCenterToWingPass = 0;
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
		public static final double feederFeedForward = 0.125;
		public static final double shooterPoopSpeed = 0.5;
	}

	public static class IntakeConstants {
		public static final int intakeMotorID = 62;
		public static final int beltMotorID = 61;

		public static final int solenoidID = 0;
		public static final int extraSolenoidID = 4;

		public static final int shooterSensorPWM_ID = 9;
		public static final int beltSensorPWM_ID = 1;

		public static final double beltIntakeSpeed = 1;
		public static final double intakeMotorSpeed = 1;

		public static final boolean intakeMotorInverted = true;
		public static final boolean beltMotorInverted = true;
	}

	public static class ClimberConstants {
		public static class LeftMotor {
			public static final int kId = 54;
			public static final boolean kInverted = true;

			/** should be 204.083928 */
			public static final double kGearboxRotationsToMechanismMeters = 18.1430612 / 0.0889;
			public static final double kMaxExtensionMeters = 0.6604;
			public static final double kExtensionThreshold = kMaxExtensionMeters - 0.482;
			public static final double kExtensionPower = 0.5;
			public static final double kRetractPower = 1;

			// 0.0889 meters moves
			// 18.1430612 rotations

			// 13 inches starting
			// 81 inches ending
		}

		public static class RightMotor {
			public static final int kId = 55;
			public static final boolean kInverted = false;

			public static final double kGearboxRotationsToMechanismMeters = LeftMotor.kGearboxRotationsToMechanismMeters;
			public static final double kMaxExtensionMeters = LeftMotor.kMaxExtensionMeters;
			public static final double kExtensionThreshold = LeftMotor.kExtensionThreshold;
			public static final double kExtensionPower = LeftMotor.kExtensionPower;
			public static final double kRetractPower = LeftMotor.kRetractPower;
		}

		public static class LeftBrakeSolenoid {
			public static final int kId = 4;
		}

		public static class RightBrakeSolenoid {
			public static final int kId = 7;
		}

		public static final double kDefaultStatorCurrentLimit = 120;
		public static final double kHomingCurrentLimit = 12;
		public static final double kHomingPower = -0.2;
		public static final double kHardStopPositionRelativeToSwitchMeters = -0.02;
	}

	public static class Lights {
		public static enum LEDState {
			/** No lights */
			kOff(new int[] { 0, 0, 0 }),

			/** Ready To Shoot */
			kGreen(new int[] { 0, 255, 0 }),
			kGreenBlink(new int[] { 0, 255, 0 }),

			/** Alliance Color */
			kRed(new int[] { 255, 0, 0 }),

			/** Alliance Color */
			kBlue(new int[] { 0, 0, 255 }),

			kYellowRed(new int[] { 255, 255, 0 }),

			kRobostangsOrange(new int[] { 255, 110, 0 }),

			kPurple(new int[] { 128, 0, 128 }),

			kBrown(new int[] { 165, 42, 42 }),

			kWhite(new int[] { 255, 255, 255 }),

			kPink(new int[] { 255, 192, 203 }),

			kCustom(new int[] { 0, 0, 0 });

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

		public static final int CANdleID = 2;
		public static final double lowVoltageThreshold = 12.5;

		/** Right Shooter Bar */
		public static final int strip1Length = 40;

		/** Left Shooter Bar */
		public static final int strip2Length = 29;

		/** Left Climber Support */
		public static final int strip3Length = 16;

		/** Right Climber Support */
		public static final int strip4Length = 17;

		public static final double animationSpeed = 0.06;
	}
}
