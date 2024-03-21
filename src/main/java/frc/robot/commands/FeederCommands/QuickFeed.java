package frc.robot.commands.FeederCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class QuickFeed extends Command {
    Intake intake;
    Shooter shooter;

    public QuickFeed() {
        intake = Intake.getInstance();
        shooter = Shooter.getInstance();

        this.addRequirements(intake, shooter);
        this.setName("Quick Feed");
    }

    @Override
    public void execute() {
        SmartDashboard.putString("Intake/Status", "Quick Feed");
        SmartDashboard.putString("Shooter/Status", "Quick Feed");
        
        intake.setBelt(0.5);
        shooter.setFeederMotor(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setBelt(0);
        shooter.setFeederMotor(0);
    }
}
