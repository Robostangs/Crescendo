package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.LoggyThings.LoggyCANcoder;
import frc.robot.LoggyThings.LoggyTalonFX;
import frc.robot.commands.shooter.FineAdjust;
import frc.robot.subsystems.Drivetrain.Drivetrain;

/**
 * To zero: first put the arm in a position where the shooter is parallel to the
 * ground, then, in tuner X, zero the armCoder
 */
public class Arm extends SubsystemBase {
    private LoggyTalonFX armMotor;
    private LoggyCANcoder armCoder;
    private double shooterExtensionSetpoint = 0;

    private double armPosition;

    public boolean ArmIsBroken = false;

    private MotionMagicTorqueCurrentFOC motionMagicDutyCycle;

    private Mechanism2d armMechanism, simArmMechanism;
    private MechanismLigament2d shooterLigament, shooterExtension, elbowLigament;
    private MechanismLigament2d simShooterLigament, simShooterExtension, simElbowLigament;

    @Override
    public void periodic() {
        updateArmPosition();

        if (Robot.isReal()) {
            shooterExtension.setAngle(getShooterExtensionPosition() - 90);
            simShooterExtension.setAngle(
                    getExtensionFromArmPosition(
                            Units.rotationsToDegrees(armMotor.getClosedLoopReference().getValueAsDouble())) - 90);
        } else {
            simShooterExtension.setAngle(shooterExtensionSetpoint - 90);

        }

        // if (Robot.isReal()) {
        SmartDashboard.putBoolean("Arm/At Setpoint", isInRangeOfTarget(getArmTarget()));
        // }

        SmartDashboard.putNumber("Arm/Arm Position", getArmPosition());
        SmartDashboard.putNumber("Arm/Velocity", getVelocityRotations());
        SmartDashboard.putNumber("Arm/Setpoint", Units.rotationsToDegrees(motionMagicDutyCycle.Position));
        SmartDashboard.putNumber("Arm/Position Error",
                Units.rotationsToDegrees(motionMagicDutyCycle.Position - getArmPosition()));
        SmartDashboard.putNumber("Arm/Calculated Setpoint", calculateArmSetpoint());

        if (Shooter.getInstance().getCurrentCommand() == null) {
            motionMagicDutyCycle.FeedForward = Constants.ArmConstants.kFeedForwardTorqueCurrent;
        } else {
            motionMagicDutyCycle.FeedForward = Constants.ArmConstants.kFeedForwardTorqueCurrentWhileShooting;
        }

        motionMagicDutyCycle.FeedForward *= Rotation2d.fromDegrees(getShooterExtensionPosition())
                .minus(Rotation2d.fromDegrees(20)).getCos();

        if (getArmPosition() > Constants.ArmConstants.kArmMaxAngle) {
            motionMagicDutyCycle.FeedForward = -8;
        }

        if (!(this.getCurrentCommand() instanceof FineAdjust)) {
            SmartDashboard.putString("Arm/Status", "Setpoint");
            if (!ArmIsBroken) {
                armMotor.setControl(motionMagicDutyCycle);
            }
        }

        if (ArmIsBroken) {
            SmartDashboard.putString("Arm/Status", "ARM IS BROKEN");
        }
    }

    private Arm() {
        armCoder = new LoggyCANcoder(Constants.ArmConstants.armEncoderID, false);
        armCoder.setPosition(Units.degreesToRotations(Constants.ArmConstants.kArmMinAngle));
        CANcoderConfiguration armCoderConfig = new CANcoderConfiguration();

        /*
         * Do this directly on the CANcoder, so as to not reset the zero position
         */
        armCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        armCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        armCoderConfig.MagnetSensor.MagnetOffset = -0.339599609375;
        armCoderConfig.MagnetSensor.MagnetOffset = 0;

        /* Do not apply */
        armCoder.getConfigurator().apply(armCoderConfig);

        // armCoder.setPosition(Units.degreesToRotations(Constants.ArmConstants.kArmMinAngle));
        // armCoder.setPosition(Units.degreesToRotations(0));

        armMotor = new LoggyTalonFX(Constants.ArmConstants.armMotorID, false);

        setArmMotorConfig(getArmMotorConfig());

        Music.getInstance().addFalcon(armMotor);

        double shooterHeightInches = 22;
        double shooterLengthInches = 9.5;
        double shooterExtensionInches = 4.5;
        double shooterExtensionAngle = -90;
        double elbowLigamentAngle = -Constants.ArmConstants.shooterOffset;
        double rootXInches = Units.metersToInches(0.7493);
        double rootYInches = 4;

        if (Robot.isReal()) {
            /* Create Motor Profile Mechanism */
            armMechanism = new Mechanism2d(0.7493 * 2, Units.inchesToMeters(
                    rootYInches + shooterHeightInches + shooterExtensionInches + shooterLengthInches + 5));

            shooterLigament = new MechanismLigament2d("Shooter Bars", Units.inchesToMeters(shooterHeightInches), 90, 9,
                    new Color8Bit(Color.kWhite));

            shooterExtension = new MechanismLigament2d("Shooter Extension",
                    Units.inchesToMeters(shooterExtensionInches), shooterExtensionAngle, 2, new Color8Bit(Color.kGray));

            elbowLigament = new MechanismLigament2d("Shooter", Units.inchesToMeters(shooterLengthInches),
                    elbowLigamentAngle, 15, new Color8Bit(Color.kWhite));

            armMechanism.getRoot("Root", Units.inchesToMeters(rootXInches), Units.inchesToMeters(rootYInches))
                    .append(shooterLigament).append(shooterExtension).append(elbowLigament);
            SmartDashboard.putData("Arm/Motor Profile", armMechanism);
        }

        /* Create Target Mechanism */
        simArmMechanism = new Mechanism2d(0.7493 * 2, Units
                .inchesToMeters(rootYInches + shooterHeightInches + shooterExtensionInches + shooterLengthInches + 5));

        simShooterLigament = new MechanismLigament2d("Shooter Bars", Units.inchesToMeters(shooterHeightInches), 90, 9,
                new Color8Bit(Color.kRed));

        simShooterExtension = new MechanismLigament2d("Shooter Extension", Units.inchesToMeters(shooterExtensionInches),
                shooterExtensionAngle, 2, new Color8Bit(Color.kDarkRed));

        simElbowLigament = new MechanismLigament2d("Shooter", Units.inchesToMeters(shooterLengthInches),
                elbowLigamentAngle, 15, new Color8Bit(Color.kRed));

        simArmMechanism.getRoot("Root", Units.inchesToMeters(rootXInches), Units.inchesToMeters(rootYInches))
                .append(simShooterLigament).append(simShooterExtension).append(simElbowLigament);
        SmartDashboard.putData("Arm/Target Profile", simArmMechanism);

        motionMagicDutyCycle = new MotionMagicTorqueCurrentFOC(
                Units.degreesToRotations(Constants.ArmConstants.kArmMinAngle));
        motionMagicDutyCycle.Slot = 0;

        BaseStatusSignal.setUpdateFrequencyForAll(50, armMotor.getClosedLoopError(), armMotor.getClosedLoopReference(),
                armMotor.getClosedLoopReferenceSlope());

        armMotor.setPosition(Units.degreesToRotations(Constants.ArmConstants.kArmMinAngle));
    }

    /**
     * whether the arm is within {@link Constants.ArmConstants#kInRangeThreshold}
     * degrees of the target setpoint
     * 
     * @param target target position in degrees
     * @return true if in range or false if out of range
     */
    public boolean isInRangeOfTarget(double target) {
        return isInRangeOfTarget(target, Constants.ArmConstants.kInRangeThreshold);
    }

    /**
     * whether the arm is within the specified degrees of the target setpoint
     * 
     * @param target         target position in degrees
     * @param rangeThreshold the threshold in degrees
     * @return true if in range or false if out of range
     */
    public boolean isInRangeOfTarget(double target, double rangeThreshold) {
        if (Math.abs((getArmPosition() - target)) < rangeThreshold) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Whether or not the Arm is within the allowable range of motion
     * 
     * @param setpoint the Shooter setpoint in degrees
     * @return true if within range, false if not
     */
    public boolean validSetpoint(double setpoint) {
        if (setpoint >= Constants.ArmConstants.kArmMinAngle && setpoint <= Constants.ArmConstants.kArmMaxAngle) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * supply a specific value [-1,1] to the arm motor, has a deadzone of 0.06
     * 
     * @param setVal the value to set the arm motor to [-1,1]
     */
    public void aim(double setVal) {
        if (!(Math.abs(setVal) <= Constants.OperatorConstants.kManipDeadzone)) {
            armMotor.set(setVal);
        }
    }

    /**
     * supply a specific value [-1,1] to the arm motor
     * 
     * @param setVal the value to set the arm motor to [-1,1]
     */
    public void aimRaw(double setVal) {
        armMotor.set(setVal);
    }

    /**
     * Returns the position of the arm in degrees
     * 
     * @return position in degrees
     */
    public double getArmPosition() {
        // if (Robot.isSimulation()) {
        // return simShooterExtension.getAngle() - Constants.ArmConstants.shooterOffset
        // + 90;
        // }
        // return Units.rotationsToDegrees(armMotor.getPosition().getValueAsDouble());

        return armPosition;
    }

    private void updateArmPosition() {
        if (Robot.isSimulation()) {
            armPosition = simShooterExtension.getAngle() - Constants.ArmConstants.shooterOffset + 90;
        } else {
            armPosition = Units.rotationsToDegrees(armMotor.getPosition().getValueAsDouble());
        }
    }

    public double getShooterExtensionPosition() {
        return getArmPosition() + Constants.ArmConstants.shooterOffset;
    }

    /**
     * Sets the Sim Arm Target Mechanism to a specific position
     * 
     * @param target in degrees
     */
    public void setArmTarget(double target) {
        shooterExtensionSetpoint = getExtensionFromArmPosition(target);
    }

    public double getArmTarget() {
        return shooterExtensionSetpoint - Constants.ArmConstants.shooterOffset;
    }

    public double getVelocityRotations() {
        return armMotor.getVelocity().getValueAsDouble();
    }

    public double getVelocity() {
        if (Robot.isSimulation()) {
            return 0;
        }

        return armMotor.getVelocity().getValueAsDouble() * 360;
    }

    /**
     * Validifies the arm setpoint and sets the arm to the setpoint
     * 
     * @param position in degrees of the arm
     */
    public void setMotionMagic(double position) {
        if (!validSetpoint(position)) {
            if (position < Constants.ArmConstants.kArmMinAngle) {
                position = Constants.ArmConstants.kArmMinAngle;
            } else {
                System.out.println(position + " is not a valid setpoint");
                position = getArmPosition();
            }
        }

        motionMagicDutyCycle.Position = Units.degreesToRotations(position);
        setArmTarget(position);
    }

    /**
     * @deprecated
     *             do not use this, use {@link #calculateArmSetpoint()} instead
     * 
     */
    public void roundArmSetpoint() {
        /* Swerve Pose calculated in meters */
        double returnVal = Constants.ArmConstants.SetPoints.kSubwoofer;
        Pose2d currentPose = Drivetrain.getInstance().getState().Pose;
        double distToSpeaker = Math.sqrt(Math.pow(currentPose.getX(), 2) + Math.pow(currentPose.getY(), 2));

        if (distToSpeaker < 4) {
            returnVal = Constants.ArmConstants.SetPoints.kSubwoofer;
        } else if (distToSpeaker < 6) {
            returnVal = Constants.ArmConstants.SetPoints.kSpeaker1;
        } else if (distToSpeaker < 8) {
            returnVal = Constants.ArmConstants.SetPoints.kSpeaker2;
        } else {
            returnVal = Constants.ArmConstants.SetPoints.kSpeaker3;
        }

        setMotionMagic(returnVal);
    }

    /**
     * Calculates the arm setpoint based on the current robot pose
     * 
     * @return the desired arm angle
     *         <h1>(in degrees)</h1>
     *         to shoot into the speaker
     */
    public double calculateArmSetpoint() {
        Pose2d speakerPose;

        if (Robot.isRed()) {
            speakerPose = Constants.Vision.SpeakerPoseRed;
        } else {
            speakerPose = Constants.Vision.SpeakerPoseBlue;
        }

        /* Swerve Pose calculated in meters */
        Pose2d currentPose = Drivetrain.getInstance().getPose();
        double SpeakerY = speakerPose.getY();

        Robot.teleopField.getObject("Speaker")
                .setPose(new Pose2d(speakerPose.getX(), SpeakerY, Rotation2d.fromDegrees(0)));

        double distToSpeakerMeters = Math.sqrt(
                Math.pow(speakerPose.getX() - currentPose.getX(), 2)
                        + Math.pow(SpeakerY - currentPose.getY(), 2));

        // double angleToSpeaker = Constants.ArmConstants.a *
        // Math.pow(Units.metersToFeet(distToSpeakerMeters), Constants.ArmConstants.b) +
        // Constants.ArmConstants.c;
        double angleToSpeaker = -1170.66 * Math.pow(distToSpeakerMeters * 39.37, -0.751072) + 1.98502;

        SmartDashboard.putNumber("Arm/Distance From Speaker (Meters)", distToSpeakerMeters);
        SmartDashboard.putNumber("Arm/Distance From Speaker (Inches)", Units.metersToInches(distToSpeakerMeters));

        /*
         * Make sure that we dont accidentally return a stupid value
         */
        if (validSetpoint(angleToSpeaker)) {
            return angleToSpeaker;
        } else {
            // System.out.println(angleToSpeaker + " is not a valid setpoint");
            if (angleToSpeaker < Constants.ArmConstants.SetPoints.kSubwoofer) {
                return Constants.ArmConstants.SetPoints.kSubwoofer;
            } else {
                return getArmPosition();
            }
        }
    }

    public double getArmPositionFromExtension(double extension) {
        return extension - Constants.ArmConstants.shooterOffset;
    }

    public double getExtensionFromArmPosition(double armPosition) {
        return armPosition + Constants.ArmConstants.shooterOffset;
    }

    public void setBrake(boolean brake) {
        NeutralModeValue mode = NeutralModeValue.Coast;
        if (brake) {
            mode = NeutralModeValue.Brake;
        }

        armMotor.setNeutralMode(mode);
    }

    public TalonFXConfiguration getArmMotorConfig() {
        TalonFXConfiguration armMotorConfig = new TalonFXConfiguration();
        armMotorConfig.Feedback.SensorToMechanismRatio = 100;

        // armMotorConfig.Feedback.RotorToSensorRatio = 50;
        // armMotorConfig.Feedback.SensorToMechanismRatio = 2;
        // armMotorConfig.Feedback.FeedbackSensorSource =
        // FeedbackSensorSourceValue.SyncCANcoder;
        // armMotorConfig.Feedback.FeedbackRemoteSensorID = armCoder.getDeviceID();

        armMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        armMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        armMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units
                .degreesToRotations(Constants.ArmConstants.kArmMaxAngle);
        armMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units
                .degreesToRotations(Constants.ArmConstants.kArmMinAngle);

        MotionMagicConfigs motionMagicConfigs = armMotorConfig.MotionMagic;

        // TODO: tune these values for less ocsilation

        armMotorConfig.Slot0.kP = 500;
        armMotorConfig.Slot0.kI = 0.03;

        armMotorConfig.Slot1.kP = 0;
        armMotorConfig.Slot1.kI = 0;

        // armMotorConfig.Slot0.kA = 0.0;
        // armMotorConfig.Slot0.kD = 0.0;
        // armMotorConfig.Slot0.kG = 0.0;
        // armMotorConfig.Slot0.kI = 5;
        // armMotorConfig.Slot0.kP = 60;
        // armMotorConfig.Slot0.kS = 0;
        // armMotorConfig.Slot0.kV = 04;

        // going higher causes oscillation
        motionMagicConfigs.MotionMagicCruiseVelocity = 0.75;
        motionMagicConfigs.MotionMagicAcceleration = 0.5;

        // tune this so that the arm starts moving quicker, 100 -> 1000
        motionMagicConfigs.MotionMagicJerk = 0;

        armMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        armMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        armMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        armMotorConfig.Audio.AllowMusicDurDisable = true;

        return armMotorConfig;
    }

    public void setArmMotorConfig(TalonFXConfiguration config) {
        armMotor.getConfigurator().apply(config);
    }

    /**
     * Will set the arm motor config
     * 
     * @param Are_You_Sure_You_Want_To_Do_This set to true in order to remove soft
     *                                         limits, set to false to enable soft
     *                                         limits
     */
    public void lastDitchEffortSetArmMotorWithoutSoftLimits(boolean Are_You_Sure_You_Want_To_Do_This) {
        if (Are_You_Sure_You_Want_To_Do_This) {
            ArmIsBroken = true;
            TalonFXConfiguration armMotorConfig = getArmMotorConfig();
            armMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
            armMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
            setArmMotorConfig(armMotorConfig);
        } else {
            ArmIsBroken = false;
            setArmMotorConfig(getArmMotorConfig());
        }
    }

    public void toggleArmMotorLimits() {
        lastDitchEffortSetArmMotorWithoutSoftLimits(!ArmIsBroken);
    }

    public boolean atSetpoint() {
        return isInRangeOfTarget(getArmTarget());
    }

    private static Arm mInstance;

    public static Arm getInstance() {
        if (mInstance == null) {
            mInstance = new Arm();
        }

        return mInstance;
    }
}