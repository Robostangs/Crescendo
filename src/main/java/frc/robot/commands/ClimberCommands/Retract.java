package frc.robot.commands.ClimberCommands;

import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj2.command.Command;

public class Retract extends Command {
	Climber climber;

	/**
	 * Retracts the climbers 
	 */
	public Retract() {
		climber = Climber.getInstance();

		this.addRequirements(climber);
		this.setName("Retract Climber");
	}

	@Override
	public void initialize() {
		climber.setLeftClimbPower(ClimberConstants.LeftMotor.kRetractPower);
		climber.setRightClimbPower(ClimberConstants.RightMotor.kRetractPower);
	}

	@Override
	public void end(boolean interrupted) {
		climber.postStatus("Retracted");
		climber.setLeftClimbPower(0);
		climber.setRightClimbPower(0);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}