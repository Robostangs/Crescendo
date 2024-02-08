package frc.robot.Subsystems;

import java.util.List;
import java.util.function.Consumer;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings("unused")
public class Music extends SubsystemBase {
    private Orchestra mOrchestra;

    private static Music mInstance;

    public static Music getInstance() {
        if (mInstance == null) {
            mInstance = new Music();
        }

        return mInstance;
    }

    public Consumer<String> queueMusic;
    private boolean playPause;

    public Music() {
        mOrchestra = new Orchestra();
        queueMusic = (chrpFile) -> mOrchestra.loadMusic(chrpFile);
        queueMusic.accept("twinkle.chrp");
        SmartDashboard.putBoolean("Music/Play Music", false);
        SmartDashboard.putString("Music/Music File", "twinkle.chrp");
        SmartDashboard.putString("Music/.type", "Aesthetic");
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
        // playPause = SmartDashboard.getBoolean("Play Music", false);

        // if (playPause && !mOrchestra.isPlaying()) {
        //     playMusic(SmartDashboard.getString("Music File", ""));
        // }

        // if (playPause) {
        //     mOrchestra.pause();
        // }
    }

    private void playMusic(String chrpFile) {
        mOrchestra.loadMusic(chrpFile);
        mOrchestra.play();
    }
}
