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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.SnapTo45;
import frc.robot.commands.SnapTo90;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOReal;
import frc.robot.subsystems.arm.ArmIOSim;
//import frc.robot.subsystems.arm.Arm;
//import frc.robot.subsystems.arm.ArmIO;
//import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTBSwerve;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
//import frc.robot.subsystems.elevator.Elevator;
//import frc.robot.subsystems.elevator.ElevatorIO;
//import frc.robot.subsystems.elevator.ElevatorIOReal;
//import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOReal;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.util.NoteVisualizer;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Elevator elevator;
  private final Arm arm;
  private final Shooter shooter;
  private final Intake intake;
  private final Indexer indexer;
  private final NoteVisualizer visualizer = new NoteVisualizer();

  private final TalonFX m_musicTalon = new TalonFX(5);

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  private final CommandXboxController driverController = new CommandXboxController(1); //Change when done
  private final CommandXboxController operatorController = new CommandXboxController(2); //Change when done

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private final Orchestra m_orchestra = new Orchestra("/home/lvuser/deploy/verySecretMusicFile.chrp");

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
        // flywheel = new Flywheel(new FlywheelIOTalonFX());
        elevator = new Elevator(new ElevatorIOReal());
        arm = new Arm(new ArmIOReal());
        shooter = new Shooter(new ShooterIOReal());
        intake = new Intake(new IntakeIOReal());
        indexer = new Indexer(new IndexerIOReal());

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
        elevator = new Elevator(new ElevatorIOSim());
        arm = new Arm(new ArmIOSim());
        shooter = new Shooter(new ShooterIOSim());
        indexer = new Indexer(new IndexerIOSim());
        intake = new Intake(new IntakeIOSim());
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
        elevator = new Elevator(new ElevatorIO() {});
        arm = new Arm(new ArmIO() {});
        shooter = new Shooter(new ShooterIO() {});
        indexer = new Indexer(new IndexerIO() {});
        intake = new Intake(new IntakeIO() {});

        break;
    }

    NoteVisualizer.setElevatorSystem(elevator);
    NoteVisualizer.setRobotPoseSupplier(drive::getPose);

    SmartDashboard.putData("Commands", CommandScheduler.getInstance());

    //NamedCommands.registerCommand("ElevatorUp", new InstantCommand(() -> elevator.setElevatorSetpoint(0.419)));
    //NamedCommands.registerCommand("ElevatorDown", new InstantCommand(() -> elevator.setElevatorSetpoint(0)));

    NamedCommands.registerCommand("Intake", new IntakeCommand(intake, indexer));
    NamedCommands.registerCommand("Shoot", new ShootCommand(shooter, drive, indexer, intake));
    NamedCommands.registerCommand("Spin Flywheels", new InstantCommand(() -> shooter.calculateShooter(drive.getDistanceFromSpeaker())).andThen(new InstantCommand(() -> shooter.setFlywheelSpeed(shooter.m_velocitySetpoint))));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    autoChooser.addOption("Flywheel Quasi Forward", shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));

    autoChooser.addOption("Flywheel Quasi Backwards", shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

    autoChooser.addOption("Flywheel Dynamic Forward", shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));

    autoChooser.addOption("Flywheel Dynamic Backwards", shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));

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
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> driverController.getRightX()));

    shooter.setDefaultCommand(new InstantCommand(() -> shooter.stopFlywheel(), shooter));
    intake.setDefaultCommand(new InstantCommand(() -> intake.stopIntake(), intake));
    indexer.setDefaultCommand(new InstantCommand(() -> indexer.stopIndexer(), indexer));
    //controller.povUp().onTrue(new InstantCommand(() -> elevator.setElevatorSetpoint(0.419)));
    //controller.povDown().onTrue(new InstantCommand(() -> elevator.setElevatorSetpoint(0)));

    //controller.b().whileTrue(new ShootCommand(shooter, drive, arm));

    controller.leftBumper().whileTrue(new IntakeCommand(intake, indexer));  
    controller.rightBumper().whileTrue(new ShootCommand(shooter, drive, indexer, intake));
    controller.povDown().whileTrue(new RunCommand(() -> intake.setIntakeSpeed(-Constants.INTAKE_SPEED), intake));
    controller.rightTrigger().whileTrue(new RunCommand(() -> intake.setIntakeSpeed(Constants.INTAKE_SPEED), intake));
    controller.povUp().whileTrue(new RunCommand(() -> indexer.setIndexerSpeed(-Constants.INDEXER_FEED_SPEED), indexer));
    controller.leftTrigger().whileTrue(new RunCommand(() -> indexer.setIndexerSpeed(Constants.INDEXER_FEED_SPEED), indexer));

    driverController.leftStick().and(driverController.rightStick()).onTrue(new InstantCommand(() -> drive.resetGyro()));

    // controller.b().whileTrue(new InstantCommand(() -> arm.setArmSetpoint(180)));
    //controller.x().whileTrue(new InstantCommand(() -> arm.setArmSetpoint(shooter.getPivotAngle())));
   
    //PANAV CONTROLS
    driverController.leftBumper().whileTrue(new IntakeCommand(intake, indexer));

    driverController.rightBumper().whileTrue(new ShootCommand(shooter, drive, indexer, intake));

    driverController.b().whileTrue(new SnapTo90(drive));

    driverController.x().whileTrue(new SnapTo45(drive));

    driverController.rightTrigger().whileTrue(
      DriveCommands.joystickDrive(
        drive,
        () -> -driverController.getLeftY(),
        () -> -driverController.getLeftX(),
        () -> 0));
  }

  public double getArmAngleDegrees() {
   return arm.getArmAngleDegrees();
  }

  public double getElevatorPositionMeters() {
   return elevator.getElevatorPositionMeters();
  }

  public void autonomousInit() {
  //  arm.seedEncoders();
  //  arm.setArmSetpoint(arm.getArmAngleDegrees());
  }

  public void teleopInit() {
  //  arm.seedEncoders();
  //  arm.setArmSetpoint(arm.getArmAngleDegrees());
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (autoChooser.get() != null) {
      return autoChooser.get();
        // .beforeStarting(new InstantCommand(() -> intake.setBarAngle(Constants.INTAKE_LOW_POSITION)));
    }
    return null;
  }
}
