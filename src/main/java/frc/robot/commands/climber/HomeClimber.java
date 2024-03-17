package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class HomeClimber extends Command {
    private final Climber mClimber;

    private HomeClimber() {
        mClimber = Climber.getInstance();
    }

    @Override
    public void initialize() {
        mClimber.setCurrentLimits(Constants.ClimberConstants.kHomingCurrentLimit);
        mClimber.setLeftClimbPower(Constants.ClimberConstants.kHomingPower);
        mClimber.setRightClimbPower(Constants.ClimberConstants.kHomingPower);
    }
    
    @Override
    public boolean isFinished() {
        return mClimber.isLeftLimitSwitchHit() && mClimber.isRightLimitSwitchHit();
    }

    @Override
    public void end(boolean interrupted) {
        mClimber.setCurrentLimits(Constants.ClimberConstants.kDefaultStatorCurrentLimit);
        mClimber.setLeftClimbPower(0);
        mClimber.setRightClimbPower(0);
        if(interrupted)
            DataLogManager.log("Climber Homing Interrupted");
    }

    public static Command getHomingCommand(){
        return new HomeClimber().withTimeout(10).withName("HomeWithTimeout");
    }
}