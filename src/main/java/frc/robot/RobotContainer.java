// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.leds.LedSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain m_MecanumSubsystem = new DriveTrain();
  @Logged
  private final ElevatorSubsystem m_Elevator = new ElevatorSubsystem();
  private final LedSubsystem m_LedSubsystem = new LedSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final ShuffleboardTab m_AutonTab = Shuffleboard.getTab("Auton");
  private final SendableChooser<Command> m_AutonChooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_codriverController =
      new CommandXboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    configureBindings();
    configureNamedCommands();

    m_AutonChooser = new SendableChooser<>();

    m_AutonChooser.setDefaultOption("None", Commands.none());
    // m_AutonChooser.addOption("NewAuton", new PathPlannerAuto("NewAuton"));

    m_AutonTab.add(m_AutonChooser);
    m_IntakeSubsystem.oneLeftBall().whileTrue(m_LedSubsystem.oneLeft());
    m_IntakeSubsystem.oneRightBall().whileTrue(m_LedSubsystem.oneRight());
    m_IntakeSubsystem.twoBalls().whileTrue(m_LedSubsystem.two());
  }

  public void configureNamedCommands() {
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
    /*
    new JoystickButton(m_driverController, Button.kA.value)
    .onTrue(
      new InstantCommand(() -> m_FourbarSubsystem.setSolenoid(Value.kForward))
    );
    */

    // m_driverController.x().onTrue(new InstantCommand(() -> m_PusherSubsystem.ejectBlock()));


    m_Elevator.setDefaultCommand(
      m_Elevator.elevatorManualCommand(
          m_codriverController::getLeftY
        )
    );

    m_codriverController.b().whileTrue(m_IntakeSubsystem.runIntake());
    m_codriverController.a().whileTrue(m_IntakeSubsystem.reverseIntake());

    m_MecanumSubsystem.setDefaultCommand(
        m_MecanumSubsystem.driveJoysticks(
            m_driverController::getLeftY, 
            m_driverController::getLeftX, 
            m_driverController::getRightX
        )
    );
  }

    
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_AutonChooser.getSelected();
  }
}
