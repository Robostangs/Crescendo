package frc.robot.commands.AutoCommands;

import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.subsystems.Drivetrain.SwerveRequest;

public class AutoManager extends Command {
    private Timer shootTimer, intakeTimer, feedTimer;
    private Intake mIntake;
    private Shooter mShooter;
    private Arm mArm;

    private String status = "Intaking";

    /**
     * this variable must push the note all the way into the shooter, touching the
     * shooter wheels,they also will be used for intaking off the floor
     */

    private static final double beltIntakeAndHandoffSpeed = 1;

    /**
     * this variable will be used to pull the note from the belt into the shooter so
     * that we can position the note
     */
    private static final double feederHandoffSpeed = 0.7;

    /**
     * the timeout that occurs when the time between activating shoot and the
     * shooter sensor being triggered is too long
     */
    private static final double shooterHandoffTimeout = 2;

    /**
     * the timeout that occurs when the shooter is taking too long to reach the
     * desired swerve rotation, shooter RPM, and arm position
     */
    private static final double shootTimeout = 3;

    /**
     * the amount of time that should be used after the shooterHandoffTimeout to
     * remove any piece stuck in the middle of the feeder in order to not aquire
     * extra points
     */
    private static final double spitTime = 0.5;

    // the piece will start reversed at the start since we put the piece in
    // properly, or we can manually reverse it, this allows for felxibility

    /**
     * These values are based on the original state of the shooter, like what do we
     * decide to be the starting position of the note
     */
    public boolean shoot = false, hasBeenReversed = true;

    public boolean shotsFired = false;

    /**
     * If we are going under stage this should defo be off, it should be on
     * regardless tho, maybe this needs to be a SendableChooser
     */
    private boolean intakeAlwaysDeployed;

    public AutoManager() {
        mIntake = Intake.getInstance();
        mShooter = Shooter.getInstance();
        mArm = Arm.getInstance();

        this.setName("Auto Manager");
        this.addRequirements(mIntake, mShooter, mArm);

        Robot.autoTab.addBoolean("Shoot", () -> shoot).withPosition(2, 0).withSize(2, 1)
                .withWidget(BuiltInWidgets.kBooleanBox);
        Robot.autoTab.addBoolean("Ready To Shoot", () -> mShooter.readyToShootAdvanced()).withPosition(2, 1)
                .withSize(2, 1).withWidget(BuiltInWidgets.kBooleanBox);
        Robot.autoTab.addString("Auto Status", () -> status).withPosition(2, 4).withSize(2, 1)
                .withWidget(BuiltInWidgets.kTextView);
    }

    @Override
    public void initialize() {
        shoot = false;
        hasBeenReversed = true;
        shotsFired = false;

        intakeAlwaysDeployed = Robot.intakeAlwaysOut.getSelected();
        shootTimer = null;

        Pose2d startPose;

        switch (Robot.startingPose.getSelected()) {
            case "amp":
                startPose = Constants.AutoConstants.WayPoints.Blue.AmpStartPosition;
                break;
            case "center":
                startPose = Constants.AutoConstants.WayPoints.Blue.CenterStartPosition;
                break;
            case "stage":
                startPose = Constants.AutoConstants.WayPoints.Blue.StageStartPosition;
                break;
            default:
                // just dont seed the pose, insted set it to be the robot pose
                System.out.println("Starting Position Undefined");
                startPose = Drivetrain.getInstance().getPose();

                // default to center start point
                // startPose = Constants.AutoConstants.WayPoints.Blue.CenterStartPosition;

                break;
        }

        if (Robot.isRed()) {
            startPose = GeometryUtil.flipFieldPose(startPose);
        }

        Drivetrain.getInstance().seedFieldRelative(startPose);
    }

    @Override
    public void execute() {
        if (intakeAlwaysDeployed) {
            mIntake.setExtend(true);
        }

        // if the shoot variable has been enabled
        if (shoot) {

            // if there is a piece in the shooter
            // if (mIntake.getShooterSensor() || Robot.isSimulation()) {
            if (mIntake.getShooterSensor()) {

                // mIntake.setHolding(true);
                intakeTimer = null;

                // reset the timer if it has not been reset
                if (feedTimer == null) {
                    feedTimer = new Timer();
                    feedTimer.restart();
                }

                // if the piece has not been reversed, and the timer indicates that it still has
                // not been reversed
                if (feedTimer.get() < Constants.ShooterConstants.feederChargeUpTime && !hasBeenReversed) {
                    shootTimer = null;
                    shotsFired = false;
                    mShooter.shoot(Constants.ShooterConstants.feederReverseFeed, 0);
                    postAutoStatus("Positioning Note to Shoot");
                }

                // wait until the shooter is ready to shoot and then shoot
                // by this point we know the piece has been reversed
                else {
                    hasBeenReversed = true;

                    // completely reset the timer
                    if (shootTimer == null) {
                        shootTimer = new Timer();
                        shootTimer.restart();
                    }

                    mArm.setMotionMagic(mArm.calculateArmSetpoint());

                    double feederSpeed = 0;

                    // if the shooter is ready to shoot, and has been ready to shoot
                    if (mShooter.readyToShootAdvanced()) {
                        feederSpeed = Constants.ShooterConstants.feederShootValue;
                        postAutoStatus("Shooting");
                        shotsFired = true;
                    } 
                    
                    // if the shooter has been trying to prepare for a while, cancel the shoot command and then just shoot
                    else if (shootTimer.get() > shootTimeout) {
                        postAutoStatus("Shooter Timed Out");
                        feederSpeed = 1;
                        shotsFired = true;
                    } 
                    
                    // if the timeout has not been triggered and the shooter is still being charged then keep charging
                    else {
                        postAutoStatus("Preparing Shooter");
                        feederSpeed = 0;
                    }

                    mShooter.shoot(feederSpeed, 1);
                }
            }

            // this says that if the shooter shooting and there is no piece in the
            // shooter, then we can go ahead and end the shoot command saying it worked
            else if (shotsFired || mShooter.readyToShoot()) {
                shoot = false;
                feedTimer = null;

                if (shootTimer.get() > shootTimeout) {
                    postAutoStatus("Shooter Timed Out");
                }

                else {
                    postAutoStatus("Successfully Shot");
                }
            }

            // if there is not a piece in the shooter and there was never anything shot out
            // of it.

            // set the shooter and belt to pass the piece into shooter, once in shooter the
            // above if statement will be triggered in the next loop will be called
            else {

                shootTimer = null;
                feedTimer = null;

                if (intakeTimer == null) {
                    intakeTimer = new Timer();
                    intakeTimer.restart();
                }

                // this code must push the piece all the way into the shooter, touching the
                // shooter wheels
                mIntake.setBelt(beltIntakeAndHandoffSpeed);
                mShooter.shoot(feederHandoffSpeed, Constants.ShooterConstants.shooterReverseSpeed);

                // if the piece is taking too long to get into the shooter, then stop trying to
                // shoot and just move on
                if (intakeTimer.get() > shooterHandoffTimeout) {

                    // we need to remove the note from the intake in this event
                    if (intakeTimer.get() > shooterHandoffTimeout + spitTime) {
                        shoot = false;
                        mIntake.setHolding(false);
                        intakeTimer = null;
                        postAutoStatus("Shooter Handoff Timed Out");
                    }

                    // removing the pieces to make sure that we dont get penalties
                    else {
                        mIntake.setBelt(-1);
                        mIntake.setIntake(-1);
                        postAutoStatus("Shooter Handoff Timed Out, Removing Note");
                    }
                }

                else {
                    postAutoStatus("Passing note to shooter");
                }
            }
        }

        // this is for when we are running standard operation (intake)
        else {
            shotsFired = false;
            mIntake.setHolding(true);
            intakeTimer = null;
            if (mIntake.getShooterSensor()) {
                // intakeTimer = null;

                // if we dont want to always have the intake deployed, then we can retract it rn
                // cuz there is a piece in the shooter
                if (!intakeAlwaysDeployed) {
                    mIntake.setExtend(false);
                }

                mIntake.setIntake(0);

                // reset the timer if it has not been reset
                if (feedTimer == null) {
                    feedTimer = new Timer();
                    feedTimer.restart();
                }

                // if the piece has not been reversed, and the timer indicates that it still has
                // not been reversed
                if (feedTimer.get() < Constants.ShooterConstants.feederChargeUpTime && !hasBeenReversed) {

                    // position the note in order to charge up the shooter motors
                    mShooter.shoot(Constants.ShooterConstants.feederReverseFeed, 0);
                    postAutoStatus("Positioning Note");
                }

                // if the piece has been reversed then start charging the shooter motors
                else {
                    hasBeenReversed = true;

                    // if this is causing problems, removing it will result in a slower auto but ig
                    // it will work at least
                    mShooter.shoot(0, 1);

                    mArm.setMotionMagic(mArm.calculateArmSetpoint());

                    // postAutoStatus("Waiting To Shoot");
                }
            }

            // if we are trying to intake and there is no piece in our shooter
            else {
                // intakeTimer = null;
                shootTimer = null;

                // always be going to intake setpoint when trying to intake
                mArm.setMotionMagic(Constants.ArmConstants.SetPoints.kIntake);

                hasBeenReversed = false;
                feedTimer = null;

                if (!intakeAlwaysDeployed) {
                    mIntake.setExtend(true);
                }
                
                mIntake.setIntake(1);

                // this code must pull the piece off the ground and into the shooter
                mIntake.setBelt(beltIntakeAndHandoffSpeed);
                mShooter.shoot(feederHandoffSpeed, Constants.ShooterConstants.shooterReverseSpeed);
                // postAutoStatus("Intaking");
            }

            shootTimer = null;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        mIntake.stop();
        mArm.setMotionMagic(Constants.ArmConstants.SetPoints.kIntake);
        Drivetrain.getInstance().setControl(new SwerveRequest.SwerveDriveBrake());
    }

    public void postAutoStatus(String status) {
        this.status = status;

        System.out.println("Auto Status @ " + Timer.getMatchTime() + ": " + status);

        if (status.contains("Time") || status.contains("time") || status.contains("Shooting")) {
            Shuffleboard.addEventMarker(status, "Auto Status @ " + Timer.getMatchTime(), EventImportance.kCritical);
        }
    }
}