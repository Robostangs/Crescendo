// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Climber;

public class AlrightTranslate extends Command {
	Climber climber;
	DoubleSupplier mLeftSupplier, mRightSupplier;

	public AlrightTranslate(DoubleSupplier leftSupplier, DoubleSupplier rightSupplier) {
		climber = Climber.getInstance();

		mLeftSupplier = leftSupplier;
		mRightSupplier = rightSupplier;

		this.addRequirements(climber);
		this.setName("Alright Translate");
	}

	@Override
	public void initialize() {
		climber.postStatus("Manually Translating");
	}

	@Override
	public void execute() {
		if (Math.abs(mLeftSupplier.getAsDouble()) > OperatorConstants.kManipDeadzone) {
			climber.setLeftClimbPower(mLeftSupplier.getAsDouble());
		}

		if (Math.abs(mRightSupplier.getAsDouble()) > OperatorConstants.kManipDeadzone) {
			climber.setRightClimbPower(mRightSupplier.getAsDouble());
		}
	}

	@Override
	public void end(boolean interrupted) {
		climber.postStatus("Idle");
		climber.setLeftClimbPower(0);
		climber.setRightClimbPower(0);
	}
}