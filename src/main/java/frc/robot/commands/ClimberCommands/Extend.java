package frc.robot.commands.ClimberCommands;

import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj2.command.Command;

public class Extend extends Command {
	Climber climber;

	/**
	 * extends the climbers based on the {@code kExtensionPower} constant
	 */
	public Extend() {
		climber = Climber.getInstance();

		this.addRequirements(climber);
		this.setName("Extend Climber");
	}


	@Override
	public void initialize() {
		climber.postStatus("Retracting");
	}

	@Override
	public void execute() {
		if (climber.getLeftPosition() < ClimberConstants.LeftMotor.kMaxExtensionMeters - ClimberConstants.LeftMotor.kExtensionThreshold) {
			climber.setLeftClimbPower(ClimberConstants.LeftMotor.kExtensionPower);
		}

		if (climber.getRightPosition() < ClimberConstants.RightMotor.kMaxExtensionMeters - ClimberConstants.RightMotor.kExtensionThreshold) {
			climber.setRightClimbPower(ClimberConstants.RightMotor.kExtensionPower);
		}
	}

	@Override
	public void end(boolean interrupted) {
		climber.setLeftClimbPower(0);
		climber.setRightClimbPower(0);
	}

	@Override
	public boolean isFinished() {
		return (climber.getLeftPosition() > ClimberConstants.LeftMotor.kMaxExtensionMeters
				- ClimberConstants.LeftMotor.kExtensionThreshold) &&
				(climber.getRightPosition() > ClimberConstants.RightMotor.kMaxExtensionMeters
						- ClimberConstants.RightMotor.kExtensionThreshold);
	}
}