package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Alert;
import frc.robot.Alert.AlertType;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.ArmCommands.FineAdjust;
import frc.robot.subsystems.Drivetrain.Drivetrain;

/**
 * To zero: first put the arm in a position where the shooter is parallel to the
 * ground, then, in tuner X, zero the armCoder
 */
public class Arm extends SubsystemBase {
    private TalonFX armMotor;
    private CANcoder armCoder;

    private double shooterExtensionSetpoint = 0;
    private double armPosition;

    public boolean ArmIsBroken = false;

    private MotionMagicTorqueCurrentFOC motionMagicDutyCycle;

    private Mechanism2d armMechanism, simArmMechanism;
    private MechanismLigament2d shooterLigament, shooterExtension, elbowLigament;
    private MechanismLigament2d simShooterLigament, simShooterExtension, simElbowLigament;

    private Alert armIsBrokenAlert = new Alert("The arm has failed!", AlertType.ERROR);

    @Override
    public void periodic() {
        updateArmPosition();

        SmartDashboard.putBoolean("Arm/At Setpoint", atSetpoint());
        SmartDashboard.putNumber("Arm/Arm Position", getArmPosition());
        SmartDashboard.putNumber("Arm/Velocity", getVelocity());
        SmartDashboard.putNumber("Arm/Setpoint", Units.rotationsToDegrees(motionMagicDutyCycle.Position));

        SmartDashboard.putNumber("Arm/Position Error",
                Units.rotationsToDegrees(motionMagicDutyCycle.Position) - getArmPosition());
        SmartDashboard.putNumber("Arm/Calculated Setpoint", calculateArmSetpoint());

        // if (Shooter.getInstance().getCurrentCommand() == null) {
            motionMagicDutyCycle.FeedForward = Constants.ArmConstants.kFeedForwardTorqueCurrent;
        // }

        // else {
        //     motionMagicDutyCycle.FeedForward = Constants.ArmConstants.kFeedForwardTorqueCurrentWhileShooting;
        // }

        motionMagicDutyCycle.FeedForward *= Rotation2d.fromDegrees(getShooterExtensionPosition())
                .minus(Rotation2d.fromDegrees(30)).getCos();

        if (getArmPosition() > Constants.ArmConstants.kArmMaxAngle) {
            motionMagicDutyCycle.FeedForward = -8;
        }

        if (!(this.getCurrentCommand() instanceof FineAdjust)) {
            // postStatus("Setpoint");
            if (!ArmIsBroken) {
                armMotor.setControl(motionMagicDutyCycle);
                // armIsBrokenAlert.set(false);
            }
        }

        if (ArmIsBroken) {
            postStatus("ARM IS BROKEN");
            // armIsBrokenAlert.set(true);
        }

        if (Robot.isReal()) {
            shooterExtension.setAngle(getShooterExtensionPosition() - 90);
            simShooterExtension.setAngle(
                    getExtensionFromArmPosition(Units.rotationsToDegrees(
                            motionMagicDutyCycle.Position)) - 90);
        }

        else {
            simShooterExtension.setAngle(shooterExtensionSetpoint - 90);
        }
    }

    private Arm() {
        armMotor = new TalonFX(Constants.ArmConstants.armMotorID, "rio");
        armCoder = new CANcoder(Constants.ArmConstants.armCoderID, "rio");

        if (Robot.verifyCANcoder(armCoder)) {
            new Alert("Arm CANcoder is not functioning, falling back onto internal encoder", AlertType.ERROR).set(true);
            var txConfig = getArmMotorConfig();
            txConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
            txConfig.Feedback.RotorToSensorRatio = 1;
            txConfig.Feedback.SensorToMechanismRatio = 100;
            applyArmMotorConfig(txConfig);
            armMotor.setPosition(Units.degreesToRotations(Constants.ArmConstants.kArmMinAngle));
        }

        else {
            applyArmMotorConfig(getArmMotorConfig());
        }

        Music.getInstance().addFalcon(armMotor);

        double shooterHeightInches = 22;
        double shooterLengthInches = 9.75;
        double shooterExtensionInches = 7.5;
        double shooterExtensionAngle = -90;
        double elbowLigamentAngle = -Constants.ArmConstants.shooterOffset;
        double rootXInches = Units.metersToInches(0.7493);
        // double rootXInches = Units.metersToInches(0.46);
        double rootYInches = 4;

        if (Robot.isReal()) {
            /* Create Motor Profile Mechanism */
            armMechanism = new Mechanism2d(0.7493 * 2, Units.inchesToMeters(
                    rootYInches + shooterHeightInches + shooterExtensionInches + shooterLengthInches + 5));

            shooterLigament = new MechanismLigament2d("Shooter Bars", Units.inchesToMeters(shooterHeightInches), 90, 16,
                    new Color8Bit(Color.kWhite));

            shooterExtension = new MechanismLigament2d("Shooter Extension",
                    Units.inchesToMeters(shooterExtensionInches), shooterExtensionAngle, 8, new Color8Bit(Color.kGray));

            elbowLigament = new MechanismLigament2d("Shooter", Units.inchesToMeters(shooterLengthInches),
                    elbowLigamentAngle, 20, new Color8Bit(Color.kWhite));

            armMechanism.getRoot("Root", Units.inchesToMeters(rootXInches), Units.inchesToMeters(rootYInches))
                    .append(shooterLigament).append(shooterExtension).append(elbowLigament);
            SmartDashboard.putData("Arm/Motor Profile", armMechanism);
        }

        /* Create Target Mechanism */
        simArmMechanism = new Mechanism2d(0.7493 * 2, Units
                .inchesToMeters(rootYInches + shooterHeightInches + shooterExtensionInches + shooterLengthInches + 5));

        simShooterLigament = new MechanismLigament2d("Shooter Bars", Units.inchesToMeters(shooterHeightInches), 90, 16,
                new Color8Bit(Color.kRed));

        simShooterExtension = new MechanismLigament2d("Shooter Extension", Units.inchesToMeters(shooterExtensionInches),
                shooterExtensionAngle, 8, new Color8Bit(Color.kDarkRed));

        simElbowLigament = new MechanismLigament2d("Shooter", Units.inchesToMeters(shooterLengthInches),
                elbowLigamentAngle, 20, new Color8Bit(Color.kRed));

        simArmMechanism.getRoot("Root", Units.inchesToMeters(rootXInches), Units.inchesToMeters(rootYInches))
                .append(simShooterLigament).append(simShooterExtension).append(simElbowLigament);
        SmartDashboard.putData("Arm/Target Profile", simArmMechanism);

        motionMagicDutyCycle = new MotionMagicTorqueCurrentFOC(
                Units.degreesToRotations(Constants.ArmConstants.kArmMinAngle));
        motionMagicDutyCycle.Slot = 0;

        setMotionMagic(Constants.ArmConstants.kArmMinAngle);
        postStatus("Idle");

        BaseStatusSignal.setUpdateFrequencyForAll(50, armMotor.getClosedLoopError(), armMotor.getClosedLoopReference(),
                armMotor.getClosedLoopReferenceSlope());
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

    public boolean isInRangeOfTarget() {
        return isInRangeOfTarget(getArmTarget());
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

            DataLogManager.log(position + " is not a valid setpoint");
            if (position < Constants.ArmConstants.kArmMinAngle) {
                position = Constants.ArmConstants.kArmMinAngle;
            }

            else {
                position = getArmPosition();
            }
        }

        motionMagicDutyCycle.Position = Units.degreesToRotations(position);
        setArmTarget(position);
    }

    /**
     * Calculates the arm setpoint based on the current robot pose
     * 
     * @return the desired arm angle
     *         <h1>(in degrees)</h1>
     *         to shoot into the speaker
     */
    public double calculateArmSetpoint() {
        return calculateArmSetpointExpo();
    }

    /**
     * Calculates the arm setpoint based on the current robot pose using an
     * exponential model
     * 
     * @return the desired arm angle
     *         <h1>(in degrees)</h1>
     *         to shoot into the speaker
     */
    public double calculateArmSetpointExpo() {
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
                .setPose(new Pose2d(speakerPose.getX(), SpeakerY,
                        Rotation2d.fromDegrees(0)));

        double distToSpeakerMeters = Math.sqrt(
                Math.pow(speakerPose.getX() - currentPose.getX(), 2)
                        + Math.pow(SpeakerY - currentPose.getY(), 2));

        double angleToSpeaker = -6798.49 * Math.pow(Units.metersToInches(distToSpeakerMeters),
                -1.24759) + -9.7318;

        SmartDashboard.putNumber("Arm/Distance From Speaker (Meters)",
                distToSpeakerMeters);
        SmartDashboard.putNumber("Arm/Distance From Speaker (Inches)",
                Units.metersToInches(distToSpeakerMeters));

        /*
         * Make sure that we dont a][\ccidentally return a stupid value
         */
        if (validSetpoint(angleToSpeaker)) {
            return angleToSpeaker;
        }

        else {
            if (angleToSpeaker < Constants.ArmConstants.SetPoints.kSubwoofer) {
                return Constants.ArmConstants.SetPoints.kSubwoofer;
            } else {
                return getArmPosition();
            }
        }
    }

    /**
     * Calculates the arm setpoint based on the current robot pose using a
     * trigonometric model
     * 
     * @return the desired arm angle
     *         <h1>(in degrees)</h1>
     *         to shoot into the speaker
     */
    public double calculateArmSetpointTrig() {
        // double groundToShooterInches = 27;

        /* Swerve Pose calculated in meters */
        Pose2d speakerPose;

        if (Robot.isRed()) {
            speakerPose = Constants.Vision.SpeakerPoseRed;
        } else {
            speakerPose = Constants.Vision.SpeakerPoseBlue;
        }

        /** Swerve Pose calculated in meters */
        Pose2d currentPose = Drivetrain.getInstance().getPose();
        double SpeakerY = speakerPose.getY();

        Robot.teleopField.getObject("Speaker")
                .setPose(new Pose2d(speakerPose.getX(), SpeakerY,
                        Rotation2d.fromDegrees(0)));

        double distToSpeakerMeters = Math.sqrt(
                Math.pow(speakerPose.getX() - currentPose.getX(), 2)
                        + Math.pow(SpeakerY - currentPose.getY(), 2));

        double groundToShooterInches = 26 + (Units.metersToInches(distToSpeakerMeters) * (1 / 53.75));

        /* ~1.3 meters */
        double shooterToSpeakerBottomMouthMeters = Constants.Vision.SpeakerHeightMeters - Units
                .inchesToMeters(groundToShooterInches);

        double angleToSpeaker = Math.atan2(shooterToSpeakerBottomMouthMeters, distToSpeakerMeters);
        angleToSpeaker = -Units.radiansToDegrees(angleToSpeaker);

        SmartDashboard.putNumber("Arm/Distance From Speaker (Meters)",
                distToSpeakerMeters);
        SmartDashboard.putNumber("Arm/Distance From Speaker (Inches)",
                Units.metersToInches(distToSpeakerMeters));

        /* Make sure that we dont accidentally return a stupid value */
        if (validSetpoint(angleToSpeaker)) {
            return angleToSpeaker;
        } else {
            return getArmPosition();
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
        armMotorConfig.Feedback.RotorToSensorRatio = 100;
        armMotorConfig.Feedback.SensorToMechanismRatio = 1;

        armMotorConfig.Feedback.FeedbackRemoteSensorID = armCoder.getDeviceID();
        armMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        armMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        armMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        armMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units
                .degreesToRotations(Constants.ArmConstants.kArmMaxAngle);
        armMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units
                .degreesToRotations(Constants.ArmConstants.kArmMinAngle);

        MotionMagicConfigs motionMagicConfigs = armMotorConfig.MotionMagic;

        // armMotorConfig.Slot0.kP = 250;
        // armMotorConfig.Slot0.kI = 3;
        armMotorConfig.Slot0.kP = 300;
        armMotorConfig.Slot0.kI = 300;
        armMotorConfig.Slot0.kD = 60;
        armMotorConfig.Slot0.kS = 6;
        armMotorConfig.Slot0.kA = 3;

        motionMagicConfigs.MotionMagicCruiseVelocity = 1.5;
        motionMagicConfigs.MotionMagicAcceleration = 2;

        // tune this so that the arm starts moving quicker, 100 -> 1000
        motionMagicConfigs.MotionMagicJerk = 0;

        armMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        armMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        armMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        armMotorConfig.Audio.AllowMusicDurDisable = true;

        return armMotorConfig;
    }

    public void applyArmMotorConfig(TalonFXConfiguration config) {
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
        TalonFXConfiguration armMotorConfig = getArmMotorConfig();

        if (Are_You_Sure_You_Want_To_Do_This) {
            ArmIsBroken = true;
            armMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
            armMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        }

        else {
            ArmIsBroken = false;
        }

        armIsBrokenAlert.set(ArmIsBroken);

        applyArmMotorConfig(armMotorConfig);
    }

    public void toggleArmMotorLimits() {
        lastDitchEffortSetArmMotorWithoutSoftLimits(!ArmIsBroken);
    }

    // TODO: increase velocity threshold (for shooting on the fly)
    public boolean atSetpoint() {
        return isInRangeOfTarget() && Math.abs(getVelocity()) < 2;
    }

    public void postStatus(String status) {
        SmartDashboard.putString("Arm/status", status);
    }

    private static Arm mInstance;

    public static Arm getInstance() {
        if (mInstance == null) {
            mInstance = new Arm();
        }

        return mInstance;
    }
}