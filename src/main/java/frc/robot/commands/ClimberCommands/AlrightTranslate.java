// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class AlrightTranslate extends Command {
	private final Climber mClimber;
	private DoubleSupplier mLeftSupplier;
	private DoubleSupplier mRightSupplier;

	public AlrightTranslate(DoubleSupplier leftSupplier, DoubleSupplier rightSupplier) {
		mClimber = Climber.getInstance();

		mLeftSupplier = leftSupplier;
		mRightSupplier = rightSupplier;

		this.setName("Climb Time");
		this.addRequirements(mClimber);
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		mClimber.setLeftClimbPower(mLeftSupplier.getAsDouble());
		mClimber.setRightClimbPower(mRightSupplier.getAsDouble());
	}

	@Override
	public void end(boolean interrupted) {
		mClimber.setLeftClimbPower(0);
		mClimber.setRightClimbPower(0);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}