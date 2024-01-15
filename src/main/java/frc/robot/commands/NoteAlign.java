package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class NoteAlign extends Command {
    CommandSwerveDrivetrain mSwerve;
    
    public NoteAlign(CommandSwerveDrivetrain mSwerve) {
        this.mSwerve = mSwerve;
    }

    public void alignToNote(){
        
    }
}
