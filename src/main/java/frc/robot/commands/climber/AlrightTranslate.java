// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;


import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

/** An example command that uses an example subsystem. */
public class AlrightTranslate extends Command {
  private final Climber mClimber;
  private DoubleSupplier mLeftSupplier;
  private DoubleSupplier mRightSupplier;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   **/

  public AlrightTranslate(DoubleSupplier leftSupplier, DoubleSupplier rightSupplier) {
    mClimber = Climber.getInstance();

    mLeftSupplier = leftSupplier;
    mRightSupplier = rightSupplier;

   // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mClimber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mClimber.setLeftClimbPower(mLeftSupplier.getAsDouble());
    mClimber.setRightClimbPower(mRightSupplier.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      mClimber.setLeftClimbPower(0);
      mClimber.setRightClimbPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { return false; }
}