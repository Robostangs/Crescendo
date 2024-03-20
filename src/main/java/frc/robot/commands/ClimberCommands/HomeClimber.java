package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class HomeClimber extends Command {
    private final Climber mClimber;

    private HomeClimber() {
        mClimber = Climber.getInstance();
        
        this.setName("Home Climber");
        this.addRequirements(mClimber);
    }

    @Override
    public void initialize() {
        mClimber.setReverseSoftLimitState(false);
        mClimber.setCurrentLimits(Constants.ClimberConstants.kHomingCurrentLimit);
    }

    @Override
    public void execute() {
        mClimber.setLeftClimbPower(Constants.ClimberConstants.kHomingPower);
        mClimber.setRightClimbPower(Constants.ClimberConstants.kHomingPower);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        mClimber.setReverseSoftLimitState(true);
        mClimber.setCurrentLimits(Constants.ClimberConstants.kDefaultStatorCurrentLimit);
        mClimber.setLeftClimbPower(0);
        mClimber.setRightClimbPower(0);

        DataLogManager.log("Climber has been Homed");
        mClimber.setLeftPosition(Constants.ClimberConstants.kHardStopPositionRelativeToSwitchMeters);
        mClimber.setRightPosition(Constants.ClimberConstants.kHardStopPositionRelativeToSwitchMeters);
    }

    public static Command getHomingCommand() {
        return new HomeClimber().withTimeout(7);
    }
}