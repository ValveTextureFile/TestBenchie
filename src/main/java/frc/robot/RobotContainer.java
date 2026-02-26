// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.GOTO;
import frc.robot.subsystems.CalamariDegree;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController;
import java.util.concurrent.ThreadLocalRandom;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final CalamariDegree m_CalamariDegree = new CalamariDegree();

  // Xbox controller on port 0 for D-pad control
  private final XboxController m_driverController = new XboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
  // D-pad (POV) mappings:
  // Typical POV angles are 0=up, 90=right, 180=down, 270=left.

  // D-pad Up -> target 90
  new Trigger(() -> m_driverController.getPOV() == 0)
    .onTrue(m_CalamariDegree.runToDegree(90));

  // D-pad Right -> target 180
  new Trigger(() -> m_driverController.getPOV() == 90)
    .onTrue(m_CalamariDegree.runToDegree(180));

  // D-pad Down -> target 230
  new Trigger(() -> m_driverController.getPOV() == 180)
    .onTrue(m_CalamariDegree.runToDegree(230));

  // D-pad Left -> random angle from 1-150
  new Trigger(() -> m_driverController.getPOV() == 270)
    .onTrue(Commands.runOnce(() -> {
      int angle = ThreadLocalRandom.current().nextInt(1, 151);
      m_CalamariDegree.runToDegree(angle).schedule();
    }, m_CalamariDegree));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An example command will be run in autonomous
  //   // return Autos.exampleAuto(m_exampleSubsystem);
  // }
}
