package frc.robot.commands;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CalamariDegree;

public class GOTO extends Command{
private final CalamariDegree sub;
private final double deg;
    
    public GOTO(CalamariDegree subsystem, double degree) {
        this.sub = subsystem;
        this.deg = degree;
        addRequirements(subsystem);
    }
    
    @Override
    public void execute() {
        sub.runToDegree(deg);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
