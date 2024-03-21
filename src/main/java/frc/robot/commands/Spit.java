package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Spit extends Command {
    Shooter shooter;
    Intake intake;

    public Spit() {
        shooter = Shooter.getInstance();
        intake = Intake.getInstance();

        this.addRequirements(shooter, intake);
        this.setName("Spit");
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Intake/Status", "Spitting");
        SmartDashboard.putString("Shooter/Status", "Spitting");
    }

    @Override
    public void execute() {
        shooter.shoot(0.5, 0.5);

        intake.setExtend(false);
        intake.setIntake(-0.5);
        intake.setBelt(-0.5);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        intake.stop();
        intake.setHolding(false);
        
        SmartDashboard.putString("Intake/Status", "Idle");
        SmartDashboard.putString("Shooter/Status", "Idle");
    }
}