package frc.robot.commands.AutoCommands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutoManager extends Command {
    private Timer shootTimer, intakeTimer, feedTimer;
    private Intake mIntake;
    private Shooter mShooter;
    private Arm mArm;

    private String status = "Intaking";

    // this code must push the piece all the way into the shooter, touching the
    // shooter wheels, they also will be used for intaking off the floor, this means
    // I need to find the right number
    private static final double beltIntakeAndHandoffSpeed = 1;
    private static final double feederHandoffSpeed = 0.7;

    private static final double shooterHandoffTimeout = 1;
    private static final double shootTimeout = 5;

    // the piece will start reversed at the start since we put the piece in
    // properly, or we can manually reverse it, this allows for felxibility

    /**
     * These values are based on the original state of the shooter, like what do we
     * decide to be the starting position of the note
     */
    public boolean shoot = false, hasBeenReversed = true;

    /**
     * If we are going under stage this should defo be off, it should be off
     * regardless tho, maybe this needs to be a SendableChooser
     */
    private boolean intakeAlwaysDeployed;

    // private boolean holdingRing = false;
    // public boolean shoot = true;
    // private boolean hasBeenReversed = true;

    public AutoManager() {
        mIntake = Intake.getInstance();
        mShooter = Shooter.getInstance();
        mArm = Arm.getInstance();

        // holdingRing = true;
        // shoot = true;
        // hasBeenReversed = true;

        this.setName("Auto Manager");
        this.addRequirements(mIntake, mShooter, mArm);
        Robot.autoTab.addString("Auto Status", () -> status).withPosition(2, 2).withSize(2, 1);
        Robot.autoTab.addBoolean("Shoot", () -> shoot).withPosition(2, 1).withSize(1, 1).withSize(1, 1)
                .withWidget(BuiltInWidgets.kBooleanBox);
        Robot.autoTab.addBoolean("Ready To Shoot", () -> mShooter.readyToShootAdvanced()).withPosition(3, 1)
                .withSize(1, 1).withWidget(BuiltInWidgets.kBooleanBox);

        // Robot.autoTab.addPersistent("Status", () -> status);
    }

    @Override
    public void initialize() {
        hasBeenReversed = true;
        intakeAlwaysDeployed = Robot.intakeAlwaysOut.getSelected();
        shootTimer = null;
        // intakeTimer = null;

    }

    @Override
    public void execute() {
        if (intakeAlwaysDeployed) {
            mIntake.setExtend(true);
        }

        // if the shoot variable has been enabled
        if (shoot) {

            // if there is a piece in the shooter
            if (mIntake.getShooterSensor()) {
                intakeTimer = null;

                // reset the timer if it has not been reset
                if (feedTimer == null) {
                    feedTimer = new Timer();
                    feedTimer.start();
                }

                // if the piece has not been reversed, and the timer indicates that it still has
                // not been reversed
                if (feedTimer.get() < Constants.ShooterConstants.feederChargeUpTime && !hasBeenReversed) {
                    mShooter.shoot(Constants.ShooterConstants.feederReverseFeed, 0);
                    postAutoStatus("Positioning Note to Shoot");
                }

                // wait until the shooter is ready to shoot and then shoot
                // by this point we know the piece has been reversed
                else {
                    feedTimer = null;
                    hasBeenReversed = true;

                    // completely reset the timer
                    if (shootTimer == null) {
                        shootTimer = new Timer();
                        shootTimer.restart();
                    }

                    mArm.setMotionMagic(mArm.calculateArmSetpoint());

                    double feederSpeed = 0;

                    // wait until the shooter is ready to shoot and the arm is in range of the
                    // setpoint, also wait until the timeout is done to ensure that we at least spit
                    // the note out

                    if ((mShooter.readyToShootAdvanced())
                            || shootTimer.get() > shootTimeout) {

                        // if ((mShooter.readyToShoot() &&
                        // mArm.isInRangeOfTarget(mArm.calculateArmSetpoint()))
                        // || shootTimer.get() > shootTimeout) {

                        // return back to intaking status when the shot has been fired
                        shoot = false;

                        feederSpeed = 1;

                        if (shootTimer.get() > shootTimeout) {
                            // make sure that we at least shoot the piece out so that we dont get penalties
                            feederSpeed = 1;
                            postAutoStatus("Shooter Timed Out");

                        }

                        else {
                            postAutoStatus("Shooting");
                        }
                    }

                    else {
                        postAutoStatus("Preparing Shooter");
                        feederSpeed = 0;
                    }

                    mShooter.shoot(feederSpeed, 1);
                }
            }

            // set the shooter and belt to pass the piece into shooter, once in shooter the
            // above if statement will be triggered in the next loop will be called
            else {
                shootTimer = null;

                if (intakeTimer == null) {
                    intakeTimer = new Timer();
                    intakeTimer.restart();
                }

                // this code must push the piece all the way into the shooter, touching the
                // shooter wheels
                mIntake.setBelt(beltIntakeAndHandoffSpeed);
                mShooter.shoot(feederHandoffSpeed, 0);

                // if the piece is taking too long to get into the shooter, then stop trying to
                // shoot and just move on
                if (intakeTimer.get() > shooterHandoffTimeout) {
                    shoot = false;
                    intakeTimer = null;
                    postAutoStatus("Shooter Handoff Timed Out");
                }

                else {
                    postAutoStatus("Waiting for piece to shoot");
                }
            }
        }

        // this is for when we are running standard operation (intake)
        else {
            // always be going to intake when not shooting
            // mArm.setMotionMagic(Constants.ArmConstants.SetPoints.kIntake);

            if (mIntake.getShooterSensor()) {
                intakeTimer = null;

                if (!intakeAlwaysDeployed) {
                    mIntake.setExtend(false);
                    mIntake.setIntake(0);
                }

                // reset the timer if it has not been reset
                if (feedTimer == null) {
                    feedTimer = new Timer();
                    feedTimer.restart();
                }

                // if the piece has not been reversed, and the timer indicates that it still has
                // not been reversed
                if (feedTimer.get() < Constants.ShooterConstants.feederChargeUpTime && !hasBeenReversed) {
                    // TODO: this is problematic because when read properly, the feeder motor will
                    // shoot the piece out at full speed, but then in the next loop set the feed
                    // motor to reverseFeed, this only happens when the shoot timeout occurs btw
                    mShooter.shoot(Constants.ShooterConstants.feederReverseFeed, 0);
                    postAutoStatus("Positioning Note");
                }

                // if the piece has been reversed then start charging the shooter motors
                else {
                    hasBeenReversed = true;
                    feedTimer = null;

                    // if this is causing problems, removing it will result in a slower auto but ig it will work at least
                    mShooter.shoot(0, 1);

                    mArm.setMotionMagic(mArm.calculateArmSetpoint());

                    postAutoStatus("Waiting To Shoot");
                }
            }

            // if we are trying to intake and there is no piece in our shooter
            else {
                // always be going to intake setpoint when trying to intake
                mArm.setMotionMagic(Constants.ArmConstants.SetPoints.kIntake);

                hasBeenReversed = false;

                mIntake.setExtend(true);
                mIntake.setIntake(1);

                // this code must pull the piece off the ground and into the shooter
                mIntake.setBelt(beltIntakeAndHandoffSpeed);
                mShooter.shoot(feederHandoffSpeed, 0);
                // postAutoStatus("Intaking");
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // mShooter.stop();
        mIntake.stop();
        mArm.setMotionMagic(Constants.ArmConstants.SetPoints.kIntake);
    }

    public void postAutoStatus(String status) {
        this.status = status;
        NetworkTableInstance.getDefault().getTable("PathPlanner").getEntry("Auto Status").setString(status);
    }
}