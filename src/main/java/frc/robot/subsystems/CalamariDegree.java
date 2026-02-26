package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CalamariDegree extends SubsystemBase {
    private TalonFX Calamari;
    private Slot0Configs PIDConf;

    public CalamariDegree() {
        Calamari = new TalonFX(11, "rio");

        var motorOutConf = new MotorOutputConfigs();
        motorOutConf.NeutralMode = NeutralModeValue.Brake;

        PIDConf = new Slot0Configs();
        PIDConf.kP = .001d;
        PIDConf.kI = 0d;
        PIDConf.kD = 0d;

        Calamari.getConfigurator().apply(motorOutConf);
        Calamari.getConfigurator().apply(PIDConf);
    }

    public Command runToDegree(double Degree) {
        return runOnce(
            () -> {
 
            }
        );
    }
}
