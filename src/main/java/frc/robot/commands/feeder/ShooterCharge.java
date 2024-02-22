package frc.robot.commands.feeder;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterCharge extends Command {
    private final Shooter mShooter = Shooter.getInstance();
    private Supplier<Double> manualAdjust;

    public ShooterCharge(Supplier<Double> manualAdjust) {
        this.manualAdjust = manualAdjust;
        this.setName("Shooter Charge");
        this.addRequirements(mShooter);
    }

    @Override
    public void execute() {
        SmartDashboard.putString("Shooter/Status", "Manual Charging");
        mShooter.shoot(0, manualAdjust.get());
    }
}