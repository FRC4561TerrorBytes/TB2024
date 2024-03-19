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


import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AmpShoot;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ModeAlign;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.SnapTo45;
import frc.robot.commands.SnapTo90;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOReal;
import frc.robot.subsystems.arm.ArmIOSim;
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
import frc.robot.util.AllianceFlipUtil;
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

  //divides the movement by the value of drive ratio.
  private double driveRatio = 1.0;
  private boolean slowMode = false;

  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(0); //Change when done
  private final CommandXboxController operatorController = new CommandXboxController(1); //Change when done

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private static final Translation3d blueSpeaker = new Translation3d(0.225, 5.55, 2.1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

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
        intake = new Intake(new IntakeIOSim());
        indexer = new Indexer(new IndexerIOSim());
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
        intake = new Intake(new IntakeIO() {});
        indexer = new Indexer(new IndexerIO() {});
        break;
    }

    NoteVisualizer.setElevatorSystem(elevator);
    NoteVisualizer.setRobotPoseSupplier(drive::getPose);

    SmartDashboard.putData("Commands", CommandScheduler.getInstance());

    NamedCommands.registerCommand("ElevatorUp", new InstantCommand(() -> elevator.setElevatorSetpoint(0.419)));
    NamedCommands.registerCommand("ElevatorDown", new InstantCommand(() -> elevator.setElevatorSetpoint(0)));

    NamedCommands.registerCommand("Intake", new IntakeCommand(intake, indexer, arm));
    NamedCommands.registerCommand("Shoot", new ShootCommand(shooter, indexer, intake, arm, visualizer));
    NamedCommands.registerCommand("Spin Flywheels", new InstantCommand(() -> shooter.calculateShooter(drive.getDistanceFromSpeaker())).andThen(new InstantCommand(() -> shooter.setFlywheelSpeed(shooter.m_velocitySetpoint))));
    NamedCommands.registerCommand("ArmShootSetPoint", new InstantCommand(() -> arm.setArmSetpoint(-6)));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up FF characterization routines
    // autoChooser.addOption(
    //     "Drive FF Characterization",
    //     new FeedForwardCharacterization(
    //         drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity));

    autoChooser.addOption("ShootGrab", new InstantCommand(() -> arm.setArmSetpoint(-6))
      .andThen(new WaitCommand(1.5))
      .andThen(new ShootCommand(shooter, indexer, intake, arm, visualizer))
        .withTimeout(1.0)
      .andThen(DriveCommands.joystickDrive(drive, () -> -0.5, () -> 0, () -> 0))
        .withTimeout(1.5));

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
            () -> -driverController.getLeftY() / driveRatio,
            () -> -driverController.getLeftX() / driveRatio,
            () -> driverController.getRightX() / driveRatio));
    
    // Default commands
    shooter.setDefaultCommand(new InstantCommand(() -> shooter.stopFlywheel(), shooter));
    intake.setDefaultCommand(new InstantCommand(() -> intake.stopIntake(), intake));
    indexer.setDefaultCommand(new InstantCommand(() -> indexer.stopIndexer(), indexer));
    //arm.setDefaultCommand(new InstantCommand(() -> arm.stopArm(), arm));
   
    // Attempted orchestra
    // driverController.start().whileTrue(new RunCommand(() -> drive.playSound(), drive));
    // driverController.back().onTrue(new InstantCommand(() -> drive.resetTrack(), drive));

  //PANAV CONTROLS
    // Intake command
    driverController.leftBumper().toggleOnTrue(new IntakeCommand(intake, indexer, arm));

    // Toggle slow mode (default normal)
    driverController.leftTrigger().onTrue(new InstantCommand(() -> adjustDriveRatio()));

    // Run shoot command 
    driverController.rightBumper().whileTrue(new ShootCommand(shooter, indexer, intake, arm, visualizer));

    // Snap 90 and 45 bindings
    driverController.b().whileTrue(new SnapTo90(drive));
    driverController.a().whileTrue(new SnapTo45(drive));

    driverController.x().whileTrue(new AmpShoot(shooter, drive, indexer, intake, arm, visualizer));

    // Auto align based on current mode
    driverController.y().whileTrue(new ModeAlign(drive, indexer, intake, arm));

    driverController.rightStick().and(driverController.leftStick()).onTrue(new InstantCommand(() -> drive.resetGyro()));

    // Lock drive to no rotation
    driverController.rightTrigger().whileTrue(
      DriveCommands.joystickDrive(
        drive,
        () -> -driverController.getLeftY(),
        () -> -driverController.getLeftX(),
        () -> 0));

    //Drive Nudges
    driverController.povUp().whileTrue(DriveCommands.joystickDrive(drive, () -> -0.5, () -> 0.0, () -> 0.0));
    driverController.povDown().whileTrue(DriveCommands.joystickDrive(drive, () -> 0.5, () -> 0.0, () -> 0.0));
    driverController.povLeft().whileTrue(DriveCommands.joystickDrive(drive, () -> 0.0, () -> -0.5, () -> 0.0));
    driverController.povRight().whileTrue(DriveCommands.joystickDrive(drive, () -> 0.0, () -> 0.5, () -> 0.0));

    // operatorController.povUp().onTrue(new InstantCommand(
    //   () -> arm.setArmSetpoint(
    //     7.7), arm));
    operatorController.leftBumper().whileTrue(new RunCommand(() -> indexer.setIndexerSpeed(-0.2), indexer));

    operatorController.rightBumper().whileTrue(new RunCommand(() -> intake.setIntakeSpeed(-Constants.INTAKE_SPEED), intake));

  //DEEKSHI CONTROLS
    // Set elevator climbing setpoints
    // operatorController.rightTrigger().onTrue(new InstantCommand(() -> elevator.setElevatorSetpoint(0.419)));
    // operatorController.leftTrigger().onTrue(new InstantCommand(() -> elevator.setElevatorSetpoint(0)));

    // Nudge elevator 5 inches up
    // operatorController.povUp().onTrue(new InstantCommand(
    //   () -> elevator.setElevatorSetpoint(
    //     elevator.getElevatorPositionMeters() + Units.inchesToMeters(5))));

    // Nudge elevator 5 inches down
    // operatorController.povDown().onTrue(new InstantCommand(
    //   () -> elevator.setElevatorSetpoint(
    //     elevator.getElevatorPositionMeters() + Units.inchesToMeters(-5))));

    // Nudge arm 5 degrees up
    operatorController.povLeft().onTrue(new InstantCommand(
      () -> arm.setArmSetpoint(
        -6), arm));// arm.getArmAngleDegrees() + 5)));

    // Nudge arm 5 degrees down
    operatorController.povRight().onTrue(new InstantCommand(
      () -> arm.setArmSetpoint(
      Constants.ARM_STOW),arm));

    operatorController.povUp().onTrue(new InstantCommand(() -> arm.setArmSetpoint(arm.getArmEncoderRotation() + 0.5), arm));
    operatorController.povDown().onTrue(new InstantCommand(() -> arm.setArmSetpoint(arm.getArmEncoderRotation() - 0.5), arm));

    operatorController.y().onTrue(new InstantCommand(() -> arm.setArmSetpoint(-5.5), arm));

    // Mode bindings
    // operatorController.b().onTrue(new InstantCommand(
    //   () -> GameMode.getInstance().setCurrentMode(Mode.TRAP)));d

    // operatorController.x().onTrue(new InstantCommand(
    //   () -> GameMode.getInstance().setCurrentMode(Mode.SPEAKER)));

    // operatorController.a().onTrue(new InstantCommand(
    //   () -> GameMode.getInstance().setCurrentMode(Mode.AMP)));

    SmartDashboard.putData(arm);
    // operatorController.y().onTrue(new InstantCommand(() -> arm.nudge(5), arm));
  }

  public double getArmAngleDegrees() {
    Logger.recordOutput("speaker thing", drive.getPose().getTranslation().getDistance(AllianceFlipUtil.apply(blueSpeaker.toTranslation2d())));
    return arm.getArmAngleDegrees();
  }

  public void flywheelSpinup() {
    if (DriverStation.isTeleopEnabled()
      && drive.getPose()
          .getTranslation()
          .getDistance(
            AllianceFlipUtil.apply(
              blueSpeaker.toTranslation2d()))
          < Units.feetToMeters(25)
      && indexer.noteInIndexer()) {
      new RunCommand(() -> shooter.setFlywheelSpeed(10), shooter);
    }
  }

  public double getElevatorPositionMeters() {
    return elevator.getElevatorPositionMeters();
  }

  public double getIntakeAngleDegrees() {
    return shooter.getPivotAngle();
  }

  public void autonomousInit() {
    // arm.setArmSetpoint(arm.getArmAngleDegrees());
  }

  public void teleopInit() {
    // arm.setArmSetpoint(arm.getArmAngleDegrees());
  }

  public void adjustDriveRatio(){
    slowMode = !slowMode;
    if (slowMode == true){
      driveRatio = 2;
    } else {
      driveRatio = 1;
    }
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
