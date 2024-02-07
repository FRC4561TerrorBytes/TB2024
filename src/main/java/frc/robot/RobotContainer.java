// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorUpCommand;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTBSwerve;
import frc.robot.subsystems.mechanism.Elevator;
import frc.robot.subsystems.mechanism.ElevatorIO;
import frc.robot.subsystems.mechanism.ElevatorIOSim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final IntakeSubsystem m_intakeSubsystem;
  private final Elevator elevator;
  private final Arm arm;

  private final TalonFX m_musicTalon = new TalonFX(5);

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private final Orchestra m_orchestra = new Orchestra("src/main/deploy/verySecretMusicFile.chrp");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_orchestra.addInstrument(m_musicTalon);

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTBSwerve(0),
                new ModuleIOTBSwerve(1),
                new ModuleIOTBSwerve(2),
                new ModuleIOTBSwerve(3));
        // drive = new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFX(0),
        // new ModuleIOTalonFX(1),
        // new ModuleIOTalonFX(2),
        // new ModuleIOTalonFX(3));
        // flywheel = new Flywheel(new FlywheelIOTalonFX());
        m_intakeSubsystem = new IntakeSubsystem();
        elevator = new Elevator(null);
        arm = new Arm(null);

        m_intakeSubsystem.setDefaultCommand(new RunCommand(() -> m_intakeSubsystem.setRollerSpeed(0), m_intakeSubsystem));
        elevator.setDefaultCommand(new InstantCommand(() -> elevator.runElevatorWithVoltage(0.0), elevator));
        break;


      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        m_intakeSubsystem = new IntakeSubsystem();
        elevator = new Elevator(new ElevatorIOSim());
        arm = new Arm(new ArmIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        m_intakeSubsystem = new IntakeSubsystem();
        elevator = new Elevator(new ElevatorIO() {});
        arm = new Arm(new ArmIO() {});
        break;
    }

    NamedCommands.registerCommand("Elevator", new ElevatorUpCommand(elevator));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up FF characterization routines
    autoChooser.addOption(
        "Drive FF Characterization",
        new FeedForwardCharacterization(
            drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity));

    // autoChooser.addOption("Square Test", AutoBuilder.buildAuto("Square"));
   
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    controller.leftBumper().whileTrue(new RunCommand( () -> m_intakeSubsystem.setRollerSpeed(0.7), m_intakeSubsystem));
    controller.rightBumper().whileTrue(new RunCommand(() -> m_intakeSubsystem.setRollerSpeed(-0.7)));
    controller.povUp().onTrue(new InstantCommand(() -> elevator.setElevatorSetpoint(0.419)));
    controller.povDown().onTrue(new InstantCommand(() -> elevator.setElevatorSetpoint(0)));

    // controller.a().whileTrue(new InstantCommand(() -> mechanism.runArmWithVoltage(12)));
    controller.a().onTrue(new InstantCommand(() -> arm.incrementArmAngle(10)));
    controller.y().onTrue(new InstantCommand(() -> arm.decrementArmAngle(10)));
    controller.b().whileTrue(new InstantCommand(() -> arm.setArmSetpoint(180)));
    controller.x().whileTrue(new InstantCommand(() -> arm.setArmSetpoint(360)));
    // controller.y().whileTrue(new InstantCommand(() -> mechanism.setElevatorSetpoint(mechanism.getElevatorPositionMeters()))
    //   .alongWith(new InstantCommand(() -> mechanism.setArmSetpoint(mechanism.getArmAngleDegrees()))));

    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    // controller.a().whileTrue(new RunCommand(() -> m_orchestra.play()));
    // controller.y().whileTrue(new RunCommand(() -> m_orchestra.stop()));
    // controller
    //     .b()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     drive.setPose(
    //                         new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
    //                 drive)
    //             .ignoringDisable(true));
   
  }

  public double getElevatorPositionMeters() {
    return elevator.getElevatorPositionMeters();
  }

  public double getArmAngleDegrees() {
    return arm.getArmAngleDegrees();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
