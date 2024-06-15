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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AmpDrive;
import frc.robot.commands.AutoNoteAlignCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LobShootCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.Leds;
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
  private boolean autoShootToggle = false;

  public static boolean lobbing = false;

  public enum shootPositions{
    STOW(-12, 0.0),
    SUBWOOFER(-4.7, 25.0),    
    PODIUM(-8, 25.0),
    AMP(7.3, 0.0),
    STAGE(-8.9, 30.0),
    WING(-9.825, 35.0),
    CENTER_AUTO_NOTE(-8, 25.0),
    LOB(-9, 5.0),
    SOURCE_SIDE_AUTO(-9.375, 30);

    private double shootSpeed;
    private double shootAngle;
    private shootPositions(double shootAngle, double shootSpeed){
        this.shootSpeed = shootSpeed;
        this.shootAngle = shootAngle;
    }

    public double getShootSpeed(){
        return shootSpeed;
    }

    public double getShootAngle(){
        return shootAngle;
    }
}

  public static shootPositions shootEnum = shootPositions.SUBWOOFER;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    Leds.getInstance();
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

        arm = new Arm(new ArmIOReal());
        indexer = new Indexer(new IndexerIOReal());
        shooter = new Shooter(new ShooterIOReal(), indexer);
        intake = new Intake(new IntakeIOReal());
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
        arm = new Arm(new ArmIOSim());
        indexer = new Indexer(new IndexerIOSim());
        intake = new Intake(new IntakeIOSim());
        shooter = new Shooter(new ShooterIOSim(), indexer);
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
        arm = new Arm(new ArmIO() {});
        indexer = new Indexer(new IndexerIO() {});
        intake = new Intake(new IntakeIO() {});
        shooter = new Shooter(new ShooterIO() {}, indexer);
        break;
    }

    SmartDashboard.putData("Commands", CommandScheduler.getInstance());

    NamedCommands.registerCommand("Intake", new IntakeCommand(intake, indexer, arm));
    NamedCommands.registerCommand("Spin Flywheels", new InstantCommand(() -> shooter.setFlywheelSpeed(15)));
    NamedCommands.registerCommand("AutoShoot", new AutoShootCommand(arm, shooter, indexer, intake, drive));
    NamedCommands.registerCommand("AutoIntake", new AutoNoteAlignCommand(drive, intake, indexer, arm));

    NamedCommands.registerCommand("SabotageIntake", new RunCommand(() -> intake.setIntakeSpeed(0.3), intake));
    NamedCommands.registerCommand("SabotageIndexer", new RunCommand(() -> indexer.setIndexerSpeed(0.4), indexer));
    NamedCommands.registerCommand("SabotageShooter", new RunCommand(() -> shooter.setFlywheelSpeed(5), shooter));


    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up FF characterization routines
    // autoChooser.addOption(
    //     "Drive FF Characterization",
    //     new FeedForwardCharacterization(
    //         drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity));

    autoChooser.addOption("ShootGrab", new InstantCommand(() -> arm.setArmSetpoint(-6))
      .andThen(new WaitCommand(1.5))
      .andThen(new ShootCommand(shooter, indexer, intake, arm, shootPositions.SUBWOOFER))
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
            () -> -driverController.getRightX() / driveRatio));
    
    // Default commands
    shooter.setDefaultCommand(new InstantCommand(() -> shooter.idleFlywheels(shootEnum), shooter));
    intake.setDefaultCommand(new InstantCommand(() -> intake.stopIntake(), intake));
    indexer.setDefaultCommand(new InstantCommand(() -> indexer.stopIndexer(), indexer));
    // led.setDefaultCommand(new InstantCommand(() -> led.setColor(rgbValues.GREEN), led));
   
  //PANAV CONTROLS
    // Intake command
    driverController.leftBumper()
      .whileTrue(new AutoNoteAlignCommand(drive, intake, indexer, arm))
      .toggleOnFalse(new IntakeCommand(intake, indexer, arm))
      .onFalse(new InstantCommand(() -> drive.stop(), drive));

    // Run shoot command (from anywhere)
    driverController.rightBumper().and(() -> autoShootToggle)
      .whileTrue(new AutoShootCommand(arm, shooter, indexer, intake, drive));

    //Preset shooting
    driverController.rightBumper().and(() -> !autoShootToggle)
      .whileTrue(new ShootCommand(shooter, indexer, intake, arm, shootEnum));

    // Reset gyro
    driverController.rightStick()
      .and(driverController.leftStick())
      .onTrue(new InstantCommand(() -> drive.resetGyro()));

    // Auto shoot toggle
    driverController.x().onTrue(new InstantCommand(() -> autoShootToggle = !autoShootToggle)
      .alongWith(new InstantCommand(() -> Leds.getInstance().autoShoot = !Leds.getInstance().autoShoot)));

    driverController.b().whileTrue(new RunCommand(() -> indexer.setIndexerSpeed(-0.4), indexer));

    driverController.a().whileTrue(new AmpDrive(drive)).onFalse(new InstantCommand(() -> drive.stop(), drive));

    driverController.rightTrigger().whileTrue(new LobShootCommand(arm, shooter, indexer));

    //Drive Nudges
    driverController.povUp().whileTrue(DriveCommands.joystickDrive(drive, () -> -0.5, () -> 0.0, () -> 0.0));
    driverController.povDown().whileTrue(DriveCommands.joystickDrive(drive, () -> 0.5, () -> 0.0, () -> 0.0));
    driverController.povLeft().whileTrue(DriveCommands.joystickDrive(drive, () -> 0.0, () -> -0.5, () -> 0.0));
    driverController.povRight().whileTrue(DriveCommands.joystickDrive(drive, () -> 0.0, () -> 0.5, () -> 0.0));

    //DEEKSHI CONTROLS
    // Subwoofer angle
    operatorController.povLeft().onTrue(new InstantCommand(() -> shootEnum = shootPositions.SUBWOOFER)
      .andThen(new InstantCommand(() -> arm.setArmSetpoint(shootEnum.getShootAngle()))));// arm.getArmAngleDegrees() + 5)));

    // Stow arm
    operatorController.povRight().onTrue(new InstantCommand(() -> shootEnum = shootPositions.STOW)
    .andThen(new InstantCommand(() -> arm.setArmSetpoint(shootPositions.STOW.getShootAngle()),arm)));
    
    //Nudge arm up/down
    operatorController.povUp().onTrue(new InstantCommand(() -> arm.setArmSetpoint(arm.getArmEncoderRotation() + 0.25), arm));
    operatorController.povDown().onTrue(new InstantCommand(() -> arm.setArmSetpoint(arm.getArmEncoderRotation() - 0.25), arm));

    //
    operatorController.a().onTrue(new InstantCommand(() -> shootEnum = shootPositions.AMP)
      .andThen(new InstantCommand(() -> arm.setArmSetpoint(shootEnum.getShootAngle()))));

    operatorController.b().onTrue(new InstantCommand(() -> shootEnum = shootPositions.STAGE)
      .andThen(new InstantCommand(() -> arm.setArmSetpoint(shootEnum.getShootAngle()))));

    operatorController.x().onTrue(new InstantCommand(() -> shootEnum = shootPositions.WING)
      .andThen(new InstantCommand(() -> arm.setArmSetpoint(shootEnum.getShootAngle()))));    

    //Podium shot angle
    operatorController.y().onTrue(new InstantCommand(() -> shootEnum = shootPositions.PODIUM)
      .andThen(new InstantCommand(() -> arm.setArmSetpoint(shootEnum.getShootAngle()))));

    //Re seed arm with abs encoder
    operatorController.rightStick().and(operatorController.leftStick())
      .onTrue(new InstantCommand(() -> arm.seedEncoders()));

    //Outtake, out-index
    operatorController.leftBumper().whileTrue(new RunCommand(() -> indexer.setIndexerSpeed(-0.2), indexer));
    operatorController.rightBumper().whileTrue(new RunCommand(() -> intake.setIntakeSpeed(-Constants.INTAKE_SPEED), intake));

    operatorController.leftTrigger().whileTrue(new RunCommand(() -> indexer.setIndexerSpeed(0.2), indexer));

    SmartDashboard.putData(arm);
    SmartDashboard.putData(indexer);
  }

  public double getArmAngleDegrees() {
    Logger.recordOutput("Shoot Enum", shootEnum);
    Logger.recordOutput("speaker thing", drive.getPose().getTranslation().getDistance(AllianceFlipUtil.apply(blueSpeaker.toTranslation2d())));
    Logger.recordOutput("Test/arm", arm.getArmAngleDegrees());
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
    return 0.0;
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
