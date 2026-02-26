package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

// SmartDashboard removed; inputs now come from controller bindings.
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CalamariDegree extends SubsystemBase {
    private final TalonFX Calamari;
    private final Slot0Configs PIDConf;
    // removed SmartDashboard tracking

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
        // Keep sending the closed-loop setpoint while the command is active.
        // This avoids the motor controller being left without periodic updates
        // which can cause the actuator to stop immediately after a one-shot.
        return Commands.run(
            () -> {
                var rot = degree / 360.0; // degrees -> revolutions (adjust for gearing if needed)
                var request = new PositionVoltage(0).withSlot(0);
                Calamari.setControl(request.withPosition(rot));
            },
            this
        );
    }

    public Command magicToDegree(double degree) {
        // Motion magic: continuously request the target while the command is active.
        return Commands.run(
            () -> {
                var rot = degree / 360.0;
                var request = new MotionMagicVoltage(rot).withSlot(0);
                Calamari.setControl(request);
            },
            this
        );
    }

    /**
     * Immediately send a position setpoint (in degrees) to the motor controller.
     * This is useful for continuous input loops (e.g. joystick) where we don't
     * want to create/schedule a new Command each update.
     */
    public void setPositionDegrees(double degree) {
        var rot = degree / 360.0; // degrees -> revolutions
        var request = new PositionVoltage(0).withSlot(0);
        Calamari.setControl(request.withPosition(rot));
    }

    @Override
    public void periodic() {
        // No periodic dashboard-driven updates. Commands are triggered from
        // controller bindings in RobotContainer.
    }
}
