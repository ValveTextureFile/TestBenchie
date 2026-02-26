package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CalamariDegree extends SubsystemBase {
    private final TalonFX Calamari;
    private final Slot0Configs PIDConf;
    private double lastTarget = Double.NaN;

    public CalamariDegree() {
        Calamari = new TalonFX(11);

        var motorOutConf = new MotorOutputConfigs();
        motorOutConf.NeutralMode = NeutralModeValue.Brake;

        PIDConf = new Slot0Configs();
        PIDConf.kP = 0.001d;
        PIDConf.kI = 0d;
        PIDConf.kD = 0d;

        Calamari.getConfigurator().apply(motorOutConf);
        Calamari.getConfigurator().apply(PIDConf);
    }

    public Command runToDegree(double degree) {
        return Commands.runOnce(
            () -> {
                var rot = degree / 360.0; // degrees -> revolutions (adjust for gearing if needed)
                var request = new PositionVoltage(0).withSlot(0);
                Calamari.setControl(request.withPosition(rot));
            },
            this
        );
    }

    public Command magicToDegree(double degree) {
        return Commands.runOnce(
            () -> {
                var rot = degree / 360.0;
                var request = new MotionMagicVoltage(rot).withSlot(0);
                Calamari.setControl(request);
            },
            this
        );
    }

    @Override
    public void periodic() {
        double deg = SmartDashboard.getNumber("TargetDegrees", 0.0);
        if (Double.isNaN(lastTarget) || deg != lastTarget) {
            lastTarget = deg;
            runToDegree(deg).schedule();
        }
    }
}
