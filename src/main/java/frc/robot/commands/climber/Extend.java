// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj2.command.Command;

public class Extend extends Command {
  private final Climber mClimber;

  public Extend() {
    mClimber = Climber.getInstance();

    addRequirements(mClimber);
  }

  @Override
  public void initialize()
  {
    mClimber.setLeftClimbPower(Constants.ClimberConstants.LeftMotor.kExtensionPower);
    mClimber.setRightClimbPower(Constants.ClimberConstants.RightMotor.kExtensionPower);
  }

  @Override
  public void execute() {
    if(mClimber.getLeftPosition() > Constants.ClimberConstants.LeftMotor.kMaxExtensionMeters - Constants.ClimberConstants.LeftMotor.kExtensionThreshold) {
      mClimber.setLeftClimbPower(0);
    }
    
    if(mClimber.getRightPosition() > Constants.ClimberConstants.RightMotor.kMaxExtensionMeters - Constants.ClimberConstants.RightMotor.kExtensionThreshold) {
      mClimber.setRightClimbPower(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
      mClimber.setLeftClimbPower(0);
      mClimber.setRightClimbPower(0);
  }

  @Override
  public boolean isFinished() {
    return
      mClimber.getLeftPosition() > Constants.ClimberConstants.LeftMotor.kMaxExtensionMeters - Constants.ClimberConstants.LeftMotor.kExtensionThreshold &&
      mClimber.getRightPosition() > Constants.ClimberConstants.RightMotor.kMaxExtensionMeters - Constants.ClimberConstants.RightMotor.kExtensionThreshold;
  }
}