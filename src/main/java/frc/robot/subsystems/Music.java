package frc.robot.subsystems;

import java.util.List;
import java.util.function.Consumer;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Alert;
import frc.robot.Alert.AlertType;

@SuppressWarnings("unused")
public class Music extends SubsystemBase {
    private Orchestra mOrchestra;

    private static Music mInstance;

    private static Alert itWorks = new Alert("IT WORKS", AlertType.INFO);

    public static Music getInstance() {
        if (mInstance == null) {
            mInstance = new Music();
        }

        return mInstance;
    }

    public Consumer<String> queueMusic;
    private boolean play;
    // private String song;
    private SendableChooser<String> songChooser;
    private String song;
    private Music() {
        mOrchestra = new Orchestra();
        queueMusic = (chrpFile) -> mOrchestra.loadMusic(chrpFile);
        

   
        queueMusic.accept("Sith.chrp");
        queueMusic.accept("underpressure.chrp");
        queueMusic.accept("sweetcaroline.chrp");
        queueMusic.accept("anotheronebitesthedust.chrp");
        SmartDashboard.putBoolean("Music/Play Music", false);
        SmartDashboard.putString("Music/Music File", "Sith.chrp");
    }
 

    public void addFalcon(TalonFX... falcons) {
        for (TalonFX falcon : falcons) {
            mOrchestra.addInstrument(falcon);
        }
    }

    public void addFalcon(List<TalonFX> falcons) {
        for (TalonFX falcon : falcons) {
            mOrchestra.addInstrument(falcon);
        }
    }


    @Override
    public void periodic() {
 

        play = SmartDashboard.getBoolean("Play Music", false);

        if (play) {
            mOrchestra.play();
        } else {
            mOrchestra.pause();
        }
    }

    
    public void playMusic(String chrpFile) {
        if (!mOrchestra.isPlaying()){
            mOrchestra.loadMusic(chrpFile);
            mOrchestra.play();
        }

        
        if(mOrchestra.isPlaying()){
            itWorks.set(true);
        }
}
}
