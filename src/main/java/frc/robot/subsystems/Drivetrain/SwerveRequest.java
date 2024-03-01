/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
package frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;

/**
 * Container for all the Swerve Requests. Use this to find all applicable swerve
 * drive requests.
 * <p>
 * This is also an interface common to all swerve drive control requests that
 * allow the
 * request to calculate the state to apply to the modules.
 */
public interface SwerveRequest {

    /*
     * Contains everything the control requests need to calculate the module state.
     */
    public class SwerveControlRequestParameters {
        public SwerveDriveKinematics kinematics;
        public Pose2d currentPose;
        public double timestamp;
        public Translation2d[] swervePositions;
        public double updatePeriod;
    }

    /**
     * Applies this swerve request to the given modules.
     * This is typically called by the SwerveDrivetrain.
     *
     * @param parameters     Parameters the control request needs to calculate the
     *                       module state
     * @param modulesToApply Modules to which the control request is applied
     * @return Status code of sending the request
     */
    public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply);

    /**
     * Sets the swerve drive module states to point inward on the
     * robot in an "X" fashion, creating a natural brake which will
     * oppose any motion.
     */
    public class SwerveDriveBrake implements SwerveRequest {

        /**
         * The type of control request to use for the drive motor.
         */
        public SwerveModule.DriveRequestType DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
        /**
         * The type of control request to use for the steer motor.
         */
        public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.MotionMagic;

        public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {

            for (int i = 0; i < modulesToApply.length; ++i) {
                SwerveModuleState state = new SwerveModuleState(0, parameters.swervePositions[i].getAngle());
                modulesToApply[i].apply(state, DriveRequestType, SteerRequestType);
            }

            return StatusCode.OK;
        }

        /**
         * Sets the type of control request to use for the drive motor.
         *
         * @param driveRequestType The type of control request to use for the drive
         *                         motor
         * @return this request
         */
        public SwerveDriveBrake withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
            this.DriveRequestType = driveRequestType;
            return this;
        }

        /**
         * Sets the type of control request to use for the steer motor.
         *
         * @param steerRequestType The type of control request to use for the steer
         *                         motor
         * @return this request
         */
        public SwerveDriveBrake withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
            this.SteerRequestType = steerRequestType;
            return this;
        }
    }

    /**
     * Drives the swerve drivetrain in a field-centric manner.
     * <p>
     * When users use this request, they specify the direction the robot should
     * travel
     * oriented against the field, and the rate at which their robot should rotate
     * about the center of the robot.
     * <p>
     * An example scenario is that the robot is oriented to the east,
     * the VelocityX is +5 m/s, VelocityY is 0 m/s, and RotationRate is 0.5 rad/s.
     * In this scenario, the robot would drive northward at 5 m/s and
     * turn counterclockwise at 0.5 rad/s.
     */
    public class FieldCentric implements SwerveRequest {
        /**
         * The velocity in the X direction, in m/s.
         * X is defined as forward according to WPILib convention,
         * so this determines how fast to travel forward.
         */
        public double VelocityX = 0;
        /**
         * The velocity in the Y direction, in m/s.
         * Y is defined as to the left according to WPILib convention,
         * so this determines how fast to travel to the left.
         */
        public double VelocityY = 0;
        /**
         * The angular rate to rotate at, in radians per second.
         * Angular rate is defined as counterclockwise positive,
         * so this determines how fast to turn counterclockwise.
         */
        public double RotationalRate = 0;
        /**
         * The allowable deadband of the request.
         */
        public double Deadband = 0.1;
        /**
         * The rotational deadband of the request.
         */
        public double RotationalDeadband = 0.05;

        public double slowDownRate = 1;

        /**
         * The location (x,y) that the robot should rotate about.
         */
        public Translation2d centerOfRotation = Constants.SwerveConstants.centerOfRotation;

        /**
         * The type of control request to use for the drive motor.
         */
        public SwerveModule.DriveRequestType DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
        /**
         * The type of control request to use for the steer motor.
         */
        public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.MotionMagic;

        /**
         * The last applied state in case we don't have anything to drive.
         */
        protected SwerveModuleState[] m_lastAppliedState = null;

        public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
            this.VelocityX *= slowDownRate;
            this.VelocityY *= slowDownRate;
            this.RotationalRate *= slowDownRate;

            double toApplyX = VelocityX;
            double toApplyY = VelocityY;
            double toApplyOmega = RotationalRate;
            if (Math.sqrt(toApplyX * toApplyX + toApplyY * toApplyY) < Deadband) {
                toApplyX = 0;
                toApplyY = 0;
            }
            if (Math.abs(toApplyOmega) < RotationalDeadband) {
                toApplyOmega = 0;
            }

            ChassisSpeeds speeds = ChassisSpeeds
                    .discretize(ChassisSpeeds.fromFieldRelativeSpeeds(toApplyX, toApplyY, toApplyOmega,
                            parameters.currentPose.getRotation()), parameters.updatePeriod);

            var states = parameters.kinematics.toSwerveModuleStates(speeds, centerOfRotation);

            for (int i = 0; i < modulesToApply.length; ++i) {
                modulesToApply[i].apply(states[i], DriveRequestType, SteerRequestType);
            }

            return StatusCode.OK;
        }

        /**
         * Sets the velocity in the X direction, in m/s.
         * X is defined as forward according to WPILib convention,
         * so this determines how fast to travel forward.
         *
         * @param velocityX Velocity in the X direction, in m/s
         * @return this request
         */
        public FieldCentric withVelocityX(double velocityX) {
            this.VelocityX = velocityX;
            return this;
        }

        /**
         * Sets the velocity in the Y direction, in m/s.
         * Y is defined as to the left according to WPILib convention,
         * so this determines how fast to travel to the left.
         *
         * @param velocityY Velocity in the Y direction, in m/s
         * @return this request
         */
        public FieldCentric withVelocityY(double velocityY) {
            this.VelocityY = velocityY;
            return this;
        }

        /**
         * The angular rate to rotate at, in radians per second.
         * Angular rate is defined as counterclockwise positive,
         * so this determines how fast to turn counterclockwise.
         *
         * @param rotationalRate Angular rate to rotate at, in radians per second
         * @return this request
         */
        public FieldCentric withRotationalRate(double rotationalRate) {
            this.RotationalRate = rotationalRate;
            return this;
        }

        public FieldCentric withSlowDown(double slowDownRate) {
            if (slowDownRate <= 0.1) {
                slowDownRate = 0.3;
            } else {
                this.slowDownRate = slowDownRate;
            }

            return this;
        }

        public FieldCentric withCenterOfRotation(Translation2d centerOfRotation) {
            this.centerOfRotation = centerOfRotation;
            return this;
        }

        /**
         * Sets the allowable deadband of the request.
         *
         * @param deadband Allowable deadband of the request
         * @return this request
         */
        public FieldCentric withDeadband(double deadband) {
            this.Deadband = deadband;
            return this;
        }

        /**
         * Sets the rotational deadband of the request.
         *
         * @param rotationalDeadband Rotational deadband of the request
         * @return this request
         */
        public FieldCentric withRotationalDeadband(double rotationalDeadband) {
            this.RotationalDeadband = rotationalDeadband;
            return this;
        }

        /**
         * Sets the type of control request to use for the drive motor.
         *
         * @param driveRequestType The type of control request to use for the drive
         *                         motor
         * @return this request
         */
        public FieldCentric withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
            this.DriveRequestType = driveRequestType;
            return this;
        }

        /**
         * Sets the type of control request to use for the steer motor.
         *
         * @param steerRequestType The type of control request to use for the steer
         *                         motor
         * @return this request
         */
        public FieldCentric withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
            this.SteerRequestType = steerRequestType;
            return this;
        }
    }

    /**
     * Drives the swerve drivetrain in a field-centric manner, maintaining a
     * specified heading
     * angle to ensure the robot is facing the desired direction
     * <p>
     * When users use this request, they specify the direction the robot should
     * travel
     * oriented against the field, and the direction the robot should be facing.
     * <p>
     * An example scenario is that the robot is oriented to the east,
     * the VelocityX is +5 m/s, VelocityY is 0 m/s, and TargetDirection is 180
     * degrees.
     * In this scenario, the robot would drive northward at 5 m/s and
     * turn clockwise to a target of 180 degrees.
     * <p>
     * This control request is especially useful for autonomous control, where the
     * robot should be facing a changing direction throughout the motion.
     */
    public class FieldCentricFacingAngle implements SwerveRequest {
        /**
         * The velocity in the X direction, in m/s.
         * X is defined as forward according to WPILib convention,
         * so this determines how fast to travel forward.
         */
        public double VelocityX = 0;
        /**
         * The velocity in the Y direction, in m/s.
         * Y is defined as to the left according to WPILib convention,
         * so this determines how fast to travel to the left.
         */
        public double VelocityY = 0;
        /**
         * The desired direction to face.
         * 0 Degrees is defined as in the direction of the X axis.
         * As a result, a TargetDirection of 90 degrees will point along
         * the Y axis, or to the left.
         */
        public Rotation2d TargetDirection = new Rotation2d();

        /**
         * The allowable deadband of the request.
         */
        public double Deadband = 0;
        /**
         * The rotational deadband of the request.
         */
        public double RotationalDeadband = 0;

        public double slowDownRate = 1;

        public boolean babyOnBoard = false;

        /**
         * The location (x,y) that the robot should rotate about.
         */
        public Translation2d centerOfRotation = Constants.SwerveConstants.centerOfRotation;

        /**
         * The type of control request to use for the drive motor.
         */
        public SwerveModule.DriveRequestType DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
        /**
         * The type of control request to use for the steer motor.
         */
        public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.MotionMagic;

        /**
         * The PID controller used to maintain the desired heading.
         * Users can specify the PID gains to change how aggressively to maintain
         * heading.
         * <p>
         * This PID controller operates on heading radians and outputs a target
         * rotational rate in radians per second.
         */

        // TODO: tune this
        public PhoenixPIDController radianController = new PhoenixPIDController(5.0, // 1.9
                20, 0.3);

        public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
            this.VelocityX *= slowDownRate;
            this.VelocityY *= slowDownRate;

            double toApplyX = VelocityX;
            double toApplyY = VelocityY;
            
            SmartDashboard.putNumber("Target Direction Before", TargetDirection.getRadians());
            SmartDashboard.putNumber("Target Direction Before Degrees", TargetDirection.getDegrees());
            
            // 180 -> -180

            // facing left is -2pi

            double rotationRate = 0;

            // this is dealing with the rotation target being plus minus half, insted of unsigned 0 to 360
            if (Robot.isRed()) {
                double Radians = TargetDirection.getRadians();
                double PoseRadians = parameters.currentPose.getRotation().getRadians();

                Radians = Units.degreesToRadians(180 + (180 + TargetDirection.getDegrees()));
                PoseRadians = Units.degreesToRadians(180 + (180 + parameters.currentPose.getRotation().getDegrees()));

                if (Radians >= 2 * Math.PI) {
                    Radians -= 2 * Math.PI;
                }

                if (PoseRadians >= 2 * Math.PI) {
                    PoseRadians -= 2 * Math.PI;
                }

                SmartDashboard.putNumber("Target Direction", Radians);

                rotationRate = radianController.calculate(PoseRadians,
                        Radians, parameters.timestamp);
            }
            
            else {
                rotationRate = radianController.calculate(parameters.currentPose.getRotation().getRadians(),
                        TargetDirection.getRadians(), parameters.timestamp);
            }

            double toApplyOmega = rotationRate;
            if (Math.sqrt(toApplyX * toApplyX + toApplyY * toApplyY) < Deadband) {
                toApplyX = 0;
                toApplyY = 0;
            }
            if (Math.abs(toApplyOmega) < RotationalDeadband) {
                toApplyOmega = 0;
            }

            ChassisSpeeds speeds = ChassisSpeeds
                    .discretize(ChassisSpeeds.fromFieldRelativeSpeeds(toApplyX, toApplyY, toApplyOmega,
                            parameters.currentPose.getRotation()), parameters.updatePeriod);

            var states = parameters.kinematics.toSwerveModuleStates(speeds, centerOfRotation);

            for (int i = 0; i < modulesToApply.length; ++i) {
                modulesToApply[i].apply(states[i], DriveRequestType, SteerRequestType);
            }

            return StatusCode.OK;
        }

        /**
         * Sets the velocity in the X direction, in m/s.
         * X is defined as forward according to WPILib convention,
         * so this determines how fast to travel forward.
         *
         * @param velocityX Velocity in the X direction, in m/s
         * @return this request
         */
        public FieldCentricFacingAngle withVelocityX(double velocityX) {
            this.VelocityX = velocityX;
            return this;
        }

        /**
         * Sets the velocity in the Y direction, in m/s.
         * Y is defined as to the left according to WPILib convention,
         * so this determines how fast to travel to the left.
         *
         * @param velocityY Velocity in the Y direction, in m/s
         * @return this request
         */
        public FieldCentricFacingAngle withVelocityY(double velocityY) {
            this.VelocityY = velocityY;
            return this;
        }

        /**
         * Sets the desired direction to face.
         * 0 Degrees is defined as in the direction of the X axis.
         * As a result, a TargetDirection of 90 degrees will point along
         * the Y axis, or to the left.
         *
         * @param targetDirection Desired direction to face
         * @return this request
         */
        public FieldCentricFacingAngle withTargetDirection(Rotation2d targetDirection) {
            this.TargetDirection = targetDirection;
            return this;
        }

        /**
         * Sets the allowable deadband of the request.
         *
         * @param deadband Allowable deadband of the request
         * @return this request
         */
        public FieldCentricFacingAngle withDeadband(double deadband) {
            this.Deadband = deadband;
            return this;
        }

        public FieldCentricFacingAngle withSlowDown(double slowDownRate) {
            if (slowDownRate <= 0.1) {
                slowDownRate = 0.3;
            } else {
                this.slowDownRate = slowDownRate;
            }

            return this;
        }

        /**
         * Sets the rotational deadband of the request.
         *
         * @param rotationalDeadband Rotational deadband of the request
         * @return this request
         */
        public FieldCentricFacingAngle withRotationalDeadband(double rotationalDeadband) {
            this.RotationalDeadband = rotationalDeadband;
            return this;
        }

        /**
         * Sets the center of rotation to rotate around.
         *
         * @param centerOfRotation Center of rotation to rotate around
         * @return this request
         */
        public FieldCentricFacingAngle withCenterOfRotation(Translation2d centerOfRotation) {
            this.centerOfRotation = centerOfRotation;
            return this;
        }

        /**
         * Sets the type of control request to use for the drive motor.
         *
         * @param driveRequestType The type of control request to use for the drive
         *                         motor
         * @return this request
         */
        public FieldCentricFacingAngle withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
            this.DriveRequestType = driveRequestType;
            return this;
        }

        /**
         * Sets the type of control request to use for the steer motor.
         *
         * @param steerRequestType The type of control request to use for the steer
         *                         motor
         * @return this request
         */
        public FieldCentricFacingAngle withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
            this.SteerRequestType = steerRequestType;
            return this;
        }
    }

    /**
     * Does nothing to the swerve module state. This is the default state of a newly
     * created swerve drive mechanism.
     */
    public class Idle implements SwerveRequest {
        public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
            return StatusCode.OK;
        }
    }

    /**
     * Sets the swerve drive modules to point to a specified direction.
     */
    public class PointWheelsAt implements SwerveRequest {

        /**
         * The direction to point the modules toward.
         * This direction is still optimized to what the module was previously at.
         */
        public Rotation2d ModuleDirection = new Rotation2d();
        /**
         * The type of control request to use for the drive motor.
         */
        public SwerveModule.DriveRequestType DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
        /**
         * The type of control request to use for the steer motor.
         */
        public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.MotionMagic;

        public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {

            for (int i = 0; i < modulesToApply.length; ++i) {
                SwerveModuleState state = new SwerveModuleState(0, ModuleDirection);
                modulesToApply[i].apply(state, DriveRequestType, SteerRequestType);
            }

            return StatusCode.OK;
        }

        /**
         * Sets the direction to point the modules toward.
         * This direction is still optimized to what the module was previously at.
         *
         * @param moduleDirection Direction to point the modules toward
         * @return this request
         */
        public PointWheelsAt withModuleDirection(Rotation2d moduleDirection) {
            this.ModuleDirection = moduleDirection;
            return this;
        }

        /**
         * Sets the type of control request to use for the drive motor.
         *
         * @param driveRequestType The type of control request to use for the drive
         *                         motor
         * @return this request
         */
        public PointWheelsAt withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
            this.DriveRequestType = driveRequestType;
            return this;
        }

        /**
         * Sets the type of control request to use for the steer motor.
         *
         * @param steerRequestType The type of control request to use for the steer
         *                         motor
         * @return this request
         */
        public PointWheelsAt withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
            this.SteerRequestType = steerRequestType;
            return this;
        }
    }

    /**
     * Drives the swerve drivetrain in a robot-centric manner.
     * <p>
     * When users use this request, they specify the direction the robot should
     * travel
     * oriented against the robot itself, and the rate at which their
     * robot should rotate about the center of the robot.
     * <p>
     * An example scenario is that the robot is oriented to the east,
     * the VelocityX is +5 m/s, VelocityY is 0 m/s, and RotationRate is 0.5 rad/s.
     * In this scenario, the robot would drive eastward at 5 m/s and
     * turn counterclockwise at 0.5 rad/s.
     */
    public class RobotCentric implements SwerveRequest {
        /**
         * The velocity in the X direction, in m/s.
         * X is defined as forward according to WPILib convention,
         * so this determines how fast to travel forward.
         */
        public double VelocityX = 0;
        /**
         * The velocity in the Y direction, in m/s.
         * Y is defined as to the left according to WPILib convention,
         * so this determines how fast to travel to the left.
         */
        public double VelocityY = 0;
        /**
         * The angular rate to rotate at, in radians per second.
         * Angular rate is defined as counterclockwise positive,
         * so this determines how fast to turn counterclockwise.
         */
        public double RotationalRate = 0;

        /**
         * The allowable deadband of the request.
         */
        public double Deadband = 0;
        /**
         * The rotational deadband of the request.
         */
        public double RotationalDeadband = 0;

        /**
         * The type of control request to use for the drive motor.
         */
        public SwerveModule.DriveRequestType DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
        /**
         * The type of control request to use for the steer motor.
         */
        public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.MotionMagic;

        public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
            double toApplyX = VelocityX;
            double toApplyY = VelocityY;
            double toApplyOmega = RotationalRate;
            if (Math.sqrt(toApplyX * toApplyX + toApplyY * toApplyY) < Deadband) {
                toApplyX = 0;
                toApplyY = 0;
            }
            if (Math.abs(toApplyOmega) < RotationalDeadband) {
                toApplyOmega = 0;
            }
            ChassisSpeeds speeds = new ChassisSpeeds(toApplyX, toApplyY, toApplyOmega);

            var states = parameters.kinematics.toSwerveModuleStates(speeds, new Translation2d());

            for (int i = 0; i < modulesToApply.length; ++i) {
                modulesToApply[i].apply(states[i], DriveRequestType, SteerRequestType);
            }

            return StatusCode.OK;
        }

        /**
         * Sets the velocity in the X direction, in m/s.
         * X is defined as forward according to WPILib convention,
         * so this determines how fast to travel forward.
         *
         * @param velocityX Velocity in the X direction, in m/s
         * @return this request
         */
        public RobotCentric withVelocityX(double velocityX) {
            this.VelocityX = velocityX;
            return this;
        }

        /**
         * Sets the velocity in the Y direction, in m/s.
         * Y is defined as to the left according to WPILib convention,
         * so this determines how fast to travel to the left.
         *
         * @param velocityY Velocity in the Y direction, in m/s
         * @return this request
         */
        public RobotCentric withVelocityY(double velocityY) {
            this.VelocityY = velocityY;
            return this;
        }

        /**
         * The angular rate to rotate at, in radians per second.
         * Angular rate is defined as counterclockwise positive,
         * so this determines how fast to turn counterclockwise.
         *
         * @param rotationalRate Angular rate to rotate at, in radians per second
         * @return this request
         */
        public RobotCentric withRotationalRate(double rotationalRate) {
            this.RotationalRate = rotationalRate;
            return this;
        }

        /**
         * Sets the allowable deadband of the request.
         *
         * @param deadband Allowable deadband of the request
         * @return this request
         */
        public RobotCentric withDeadband(double deadband) {
            this.Deadband = deadband;
            return this;
        }

        /**
         * Sets the rotational deadband of the request.
         *
         * @param rotationalDeadband Rotational deadband of the request
         * @return this request
         */
        public RobotCentric withRotationalDeadband(double rotationalDeadband) {
            this.RotationalDeadband = rotationalDeadband;
            return this;
        }

        /**
         * Sets the type of control request to use for the drive motor.
         *
         * @param driveRequestType The type of control request to use for the drive
         *                         motor
         * @return this request
         */
        public RobotCentric withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
            this.DriveRequestType = driveRequestType;
            return this;
        }

        /**
         * Sets the type of control request to use for the steer motor.
         *
         * @param steerRequestType The type of control request to use for the steer
         *                         motor
         * @return this request
         */
        public RobotCentric withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
            this.SteerRequestType = steerRequestType;
            return this;
        }
    }

    /**
     * Accepts a generic ChassisSpeeds to apply to the drivetrain.
     */
    public class ApplyChassisSpeeds implements SwerveRequest {

        /**
         * The chassis speeds to apply to the drivetrain.
         */
        public ChassisSpeeds Speeds = new ChassisSpeeds();
        /**
         * The center of rotation to rotate around.
         */
        public Translation2d CenterOfRotation = new Translation2d(0, 0);
        /**
         * The type of control request to use for the drive motor.
         */
        public SwerveModule.DriveRequestType DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
        /**
         * The type of control request to use for the steer motor.
         */
        public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.MotionMagic;

        public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
            var states = parameters.kinematics.toSwerveModuleStates(Speeds, CenterOfRotation);
            for (int i = 0; i < modulesToApply.length; ++i) {
                modulesToApply[i].apply(states[i], DriveRequestType, SteerRequestType);
            }

            return StatusCode.OK;
        }

        /**
         * Sets the chassis speeds to apply to the drivetrain.
         *
         * @param speeds Chassis speeds to apply to the drivetrain
         * @return this request
         */
        public ApplyChassisSpeeds withSpeeds(ChassisSpeeds speeds) {
            this.Speeds = speeds;
            return this;
        }

        /**
         * Sets the center of rotation to rotate around.
         *
         * @param centerOfRotation Center of rotation to rotate around
         * @return this request
         */
        public ApplyChassisSpeeds withCenterOfRotation(Translation2d centerOfRotation) {
            this.CenterOfRotation = centerOfRotation;
            return this;
        }

        /**
         * Sets the type of control request to use for the drive motor.
         *
         * @param driveRequestType The type of control request to use for the drive
         *                         motor
         * @return this request
         */
        public ApplyChassisSpeeds withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
            this.DriveRequestType = driveRequestType;
            return this;
        }

        /**
         * Sets the type of control request to use for the steer motor.
         *
         * @param steerRequestType The type of control request to use for the steer
         *                         motor
         * @return this request
         */
        public ApplyChassisSpeeds withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
            this.SteerRequestType = steerRequestType;
            return this;
        }
    }
}