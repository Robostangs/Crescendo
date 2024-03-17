// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class Extend extends Command {
  private final Climber mClimber;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   **/

  public Extend() {
    mClimber = Climber.getInstance();

   // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mClimber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    mClimber.setLeftClimbPower(Constants.ClimberConstants.LeftMotor.kExtensionPower);
    mClimber.setRightClimbPower(Constants.ClimberConstants.RightMotor.kExtensionPower);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(mClimber.getLeftPosition() > Constants.ClimberConstants.LeftMotor.kMaxExtensionMeters - Constants.ClimberConstants.LeftMotor.kExtensionThreshold) {
      mClimber.setLeftClimbPower(0);
    }
    
    if(mClimber.getRightPosition() > Constants.ClimberConstants.RightMotor.kMaxExtensionMeters - Constants.ClimberConstants.RightMotor.kExtensionThreshold) {
      mClimber.setRightClimbPower(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      mClimber.setLeftClimbPower(0);
      mClimber.setRightClimbPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return
      mClimber.getLeftPosition() > Constants.ClimberConstants.LeftMotor.kMaxExtensionMeters - Constants.ClimberConstants.LeftMotor.kExtensionThreshold &&
      mClimber.getRightPosition() > Constants.ClimberConstants.RightMotor.kMaxExtensionMeters - Constants.ClimberConstants.RightMotor.kExtensionThreshold;
  }
}