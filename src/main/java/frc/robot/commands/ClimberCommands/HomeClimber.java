package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class HomeClimber extends Command {
    Climber climber;

    private HomeClimber() {
        climber = Climber.getInstance();
        
        this.addRequirements(climber);
        this.setName("Home Climber");
    }

    @Override
    public void initialize() {
        climber.setReverseSoftLimitState(false);
        climber.setCurrentLimits(Constants.ClimberConstants.kHomingCurrentLimit);
    }

    @Override
    public void execute() {
        climber.setLeftClimbPower(Constants.ClimberConstants.kHomingPower);
        climber.setRightClimbPower(Constants.ClimberConstants.kHomingPower);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        climber.setReverseSoftLimitState(true);
        climber.setCurrentLimits(Constants.ClimberConstants.kDefaultStatorCurrentLimit);
        climber.setLeftClimbPower(0);
        climber.setRightClimbPower(0);

        if (!interrupted) {
            DataLogManager.log("Climber has been Homed");            
        }
        
        climber.setLeftPosition(Constants.ClimberConstants.kHardStopPositionRelativeToSwitchMeters);
        climber.setRightPosition(Constants.ClimberConstants.kHardStopPositionRelativeToSwitchMeters);
    }

    public static Command getHomingCommand() {
        return new HomeClimber().withTimeout(7).withName("Home Climber 7 Second Timeout");
    }
}