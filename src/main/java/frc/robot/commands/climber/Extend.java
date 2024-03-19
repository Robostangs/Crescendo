package frc.robot.commands.climber;

import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj2.command.Command;

public class Extend extends Command {
	private final Climber mClimber;

	public Extend() {
		mClimber = Climber.getInstance();

		this.setName("Extend Climber");
		addRequirements(mClimber);
	}

	@Override
	public void execute() {
		if (mClimber.getLeftPosition() < ClimberConstants.LeftMotor.kMaxExtensionMeters - ClimberConstants.LeftMotor.kExtensionThreshold) {
			mClimber.setLeftClimbPower(ClimberConstants.LeftMotor.kExtensionPower);
		}

		if (mClimber.getRightPosition() < ClimberConstants.RightMotor.kMaxExtensionMeters - ClimberConstants.RightMotor.kExtensionThreshold) {
			mClimber.setLeftClimbPower(ClimberConstants.RightMotor.kExtensionPower);
		}
	}

	@Override
	public void end(boolean interrupted) {
		mClimber.setLeftClimbPower(0);
		mClimber.setRightClimbPower(0);
	}

	@Override
	public boolean isFinished() {
		return mClimber.getLeftPosition() > ClimberConstants.LeftMotor.kMaxExtensionMeters
				- ClimberConstants.LeftMotor.kExtensionThreshold &&
				mClimber.getRightPosition() > ClimberConstants.RightMotor.kMaxExtensionMeters
						- ClimberConstants.RightMotor.kExtensionThreshold;
	}
}