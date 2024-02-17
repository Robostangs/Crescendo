package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
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
    private static Arm mInstance;
    private LoggyTalonFX armMotor;
    private LoggyCANcoder armCoder;
    private double shooterExtensionSetpoint = 0;

    private MotionMagicTorqueCurrentFOC motionMagicDutyCycle;

    private Mechanism2d armMechanism, simArmMechanism;
    private MechanismLigament2d shooterLigament, shooterExtension, elbowLigament;
    private MechanismLigament2d simShooterLigament, simShooterExtension, simElbowLigament;

    public static Arm getInstance() {
        if (mInstance == null) {
            mInstance = new Arm();
        }

        return mInstance;
    }

    @Override
    public void periodic() {
        if (Robot.isReal()) {
            shooterExtension.setAngle(getShooterExtensionPosition() - 90);
            simShooterExtension.setAngle(
                    getExtensionFromArmPosition(Units.rotationsToDegrees(armMotor.getClosedLoopReference().getValueAsDouble())) - 90);
        }
        simShooterExtension.setAngle(shooterExtensionSetpoint - 90);

        if (Robot.isReal()) {
            if (isInRangeOfTarget(getArmTarget(), 3)) {
                elbowLigament.setColor(new Color8Bit(Color.kGreen));
                SmartDashboard.putBoolean("Arm/At Setpoint", true);
            } else {
                elbowLigament.setColor(new Color8Bit(Color.kWhite));
                SmartDashboard.putBoolean("Arm/At Setpoint", false);
            }
        }

        SmartDashboard.putString("Arm/.type", "Subsystem");
        SmartDashboard.putNumber("Arm/Arm Position", getArmPosition());
        SmartDashboard.putNumber("Arm/VelocityRPS", getVelocityRotations());
        SmartDashboard.putNumber("Arm/VelocitySetpointRPS", armMotor.getClosedLoopReferenceSlope().getValueAsDouble());
        SmartDashboard.putNumber("Arm/Setpoint",
                Units.rotationsToDegrees(armMotor.getClosedLoopReference().getValueAsDouble()));
        SmartDashboard.putNumber("Arm/Position Error",
                Units.rotationsToDegrees(armMotor.getClosedLoopError().getValueAsDouble()));
        // SmartDashboard.putNumber("Arm/Position Error",
        // Units.rotationsToDegrees(motionMagicDutyCycle.Position) - getArmPosition());
        SmartDashboard.putNumber("Arm/Target Setpoint", getArmTarget());

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
            armMotor.setControl(motionMagicDutyCycle);
        }
    }

    private Arm() {
        armCoder = new LoggyCANcoder(Constants.ArmConstants.armEncoderID, false);
        CANcoderConfiguration armCoderConfig = new CANcoderConfiguration();

        /*
         * Do this directly on the CANcoder, so as to not reset the zero position
         */
        armCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        armCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        /* Do not apply */
        // armCoder.getConfigurator().apply(armCoderConfig);

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
                    elbowLigamentAngle, 5, new Color8Bit(Color.kWhite));

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
                elbowLigamentAngle, 5, new Color8Bit(Color.kRed));

        simArmMechanism.getRoot("Root", Units.inchesToMeters(rootXInches), Units.inchesToMeters(rootYInches))
                .append(simShooterLigament).append(simShooterExtension).append(simElbowLigament);
        SmartDashboard.putData("Arm/Target Profile", simArmMechanism);

        motionMagicDutyCycle = new MotionMagicTorqueCurrentFOC(
                Units.degreesToRotations(Constants.ArmConstants.SetPoints.kSubwoofer));
        motionMagicDutyCycle.Slot = 0;
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
        if (!(Math.abs(setVal) <= Constants.OperatorConstants.kDeadzone)) {
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
        if (Robot.isSimulation()) {
            return simShooterExtension.getAngle() - Constants.ArmConstants.shooterOffset + 90;
        }
        return Units.rotationsToDegrees(armMotor.getPosition().getValueAsDouble());
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
        shooterExtensionSetpoint = target + Constants.ArmConstants.shooterOffset;
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

    public void setMotionMagic(double position) {
        motionMagicDutyCycle.Position = Units.degreesToRotations(position);
        setArmTarget(position);
    }

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

        if (validSetpoint(returnVal)) {
            setArmTarget(returnVal);
            setMotionMagic(returnVal);
        } else {
            System.out.println(returnVal + " is not a valid setpoint");
        }
    }

    public double calculateArmSetpoint() {
        double groundToShooterInches = 27;
        // double floorToSpeakerBottomMouthInches = 78;

        /* ~1.3 meters */
        double shooterToSpeakerBottomMouthMeters = Constants.Vision.SpeakerCoords[2] - Units
                .inchesToMeters(groundToShooterInches);

        /* Swerve Pose calculated in meters */
        Pose2d currentPose = Drivetrain.getInstance().getState().Pose;
        double distToSpeakerMeters = Math.sqrt(
                Math.pow(Constants.Vision.SpeakerCoords[0] - currentPose.getX(), 2)
                        + Math.pow(Constants.Vision.SpeakerCoords[1] - currentPose.getY(), 2)) - Units.inchesToMeters(15);
        // System.out.println("Distance to speaker: " + Units.metersToInches(distToSpeakerMeters));
        // System.out.println("Height to Speaker: " +
        //         Units.metersToInches(shooterToSpeakerBottomMouthMeters));

        double angleToSpeaker = Math.atan2(shooterToSpeakerBottomMouthMeters, distToSpeakerMeters);
        // System.out.println("Angle to speaker radians atan2: " + angleToSpeaker);
        angleToSpeaker = -Units.radiansToDegrees(angleToSpeaker);
        System.out.println("Angle to speaker Degrees: " + angleToSpeaker);

        /* Make sure that we dont accidentally return a stupid value */
        if (validSetpoint(angleToSpeaker)) {
            return angleToSpeaker;
        } else {
            System.out.println(angleToSpeaker + " is not a valid setpoint");
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
        armMotorConfig.Feedback.SensorToMechanismRatio = 2;
        armMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
        armMotorConfig.Feedback.RotorToSensorRatio = 50;
        armMotorConfig.Feedback.FeedbackRemoteSensorID = armCoder.getDeviceID();

        armMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        armMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        armMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units
                .degreesToRotations(Constants.ArmConstants.kArmMaxAngle);
        armMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units
                .degreesToRotations(Constants.ArmConstants.kArmMinAngle);

        MotionMagicConfigs motionMagicConfigs = armMotorConfig.MotionMagic;

        armMotorConfig.Slot0.kP = 500;
        armMotorConfig.Slot0.kI = 0.02;

        armMotorConfig.Slot1.kP = 0;
        armMotorConfig.Slot1.kI = 0;

        // armMotorConfig.Slot0.kA = 0.0;
        // armMotorConfig.Slot0.kD = 0.0;
        // armMotorConfig.Slot0.kG = 0.0;
        // armMotorConfig.Slot0.kI = 5;
        // armMotorConfig.Slot0.kP = 60;
        // armMotorConfig.Slot0.kS = 0;
        // armMotorConfig.Slot0.kV = 04;
        motionMagicConfigs.MotionMagicCruiseVelocity = 0.25;
        motionMagicConfigs.MotionMagicAcceleration = 0.5;
        /*
         * Adjust to get trapezoidal formation (use the velocity posted in
         * smartdashobard to track trapezoid)
         */
        motionMagicConfigs.MotionMagicJerk = 100;

        armMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        armMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        armMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        armMotorConfig.Audio.AllowMusicDurDisable = true;

        return armMotorConfig;
    }

    public void setArmMotorConfig(TalonFXConfiguration config) {
        armMotor.getConfigurator().apply(config);
    }

    public void lastDitchEffortSetArmMotorWithoutSoftLimits(boolean Are_You_Sure_You_Want_To_Do_This) {
        if (Are_You_Sure_You_Want_To_Do_This) {
            TalonFXConfiguration armMotorConfig = getArmMotorConfig();
            armMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
            armMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
            armMotor.getConfigurator().apply(armMotorConfig);
        }
    }
}
