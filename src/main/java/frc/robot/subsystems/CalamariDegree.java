package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CalamariDegree extends SubsystemBase {
    private TalonFX Calamari;
    private Slot0Configs PIDConf;

    public CalamariDegree() {
        Calamari = new TalonFX(11);

        var motorOutConf = new MotorOutputConfigs();
        motorOutConf.NeutralMode = NeutralModeValue.Brake;

        PIDConf = new Slot0Configs();
        PIDConf.kP = .001d;
        PIDConf.kI = 0d;
        PIDConf.kD = 0d;

        Calamari.getConfigurator().apply(motorOutConf);
        Calamari.getConfigurator().apply(PIDConf);
    }

    public Command runToDegree(double degree) {
        return runOnce(
            () -> {
                var rot = degree/360.;
                var request = new PositionVoltage(0).withSlot(0);
                Calamari.setControl(request.withPosition(rot));

            }
        );
    }

    public Command magicToDegree(double degree) {
        return runOnce(
            () -> {
                var request = new MotionMagicVoltage(degree/360.).withSlot(0);
                Calamari.setControl(request);
            }
        );
    }

    @Override
    public void periodic() {
        var degString = SmartDashboard.getString("TargetDegrees",
        "0");
        var deg = Double.parseDouble(degString);
        runToDegree(deg);
    }

    public Command magicToDegree(double degree) {
        return runOnce(
            () -> {
                var request = new MotionMagicVoltage(degree/360.).withSlot(0);
                Calamari.setControl(request);
            }
        );
    }

    @Override
    public void periodic() {
        var degString = SmartDashboard.getString("TargetDegrees",
        "0");
        var deg = Double.parseDouble(degString);
        runToDegree(deg);
    }
}
