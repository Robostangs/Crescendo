package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Spit extends Command {
    private Shooter mShooter;
    private Intake mIntake;

    public Spit() {
        mShooter = Shooter.getInstance();
        mIntake = Intake.getInstance();
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Intake/Status", "Spitting");
        SmartDashboard.putString("Shooter/Status", "Spitting");
    }

    @Override
    public void execute() {
        mShooter.shoot(0.5, 0.5);
        mIntake.setMotor(-0.5);
    }

    @Override
    public void end(boolean interrupted) {
        mShooter.shoot(0, 0);
        mIntake.setMotor(0);
        SmartDashboard.putString("Intake/Status", "Idle");
        SmartDashboard.putString("Shooter/Status", "Idle");
    }

}
