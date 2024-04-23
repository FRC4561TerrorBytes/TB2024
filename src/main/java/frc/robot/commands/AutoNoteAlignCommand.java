// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer.shootPositions;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;

public class AutoNoteAlignCommand extends Command {

  private final Drive drive;
  private final Intake intake;
  private final Indexer indexer;
  private final Arm arm;

  private Command pathfindCommand;

  double camMountHeightMeters = 20.0;
  double camMountAngleRad = Units.degreesToRadians(10);

  double camVerticalPOVRad = Units.degreesToRadians(48.9);
  double camHeightPixels = 480.0;
  double verticalAnglePerPixel = camVerticalPOVRad / camHeightPixels;

  double camHorizontalPOVRad = Units.degreesToRadians(48.9);
  double camWidthPixels = 480.0;
  double horizontalAnglePerPixel = camHorizontalPOVRad / camWidthPixels;

  public AutoNoteAlignCommand(Drive drive, Intake intake, Indexer indexer, Arm arm) {
    this.drive = drive;
    this.intake = intake;
    this.indexer = indexer;
    this.arm = arm;

    addRequirements(intake, indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.stop();
    LimelightHelpers.setLEDMode_ForceOn(Constants.DRIVER_LIMELIGHT);
    arm.setArmSetpoint(shootPositions.STOW.getShootAngle());
    Leds.getInstance().autoNoteAlign = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    NetworkTable chair = NetworkTableInstance.getDefault().getTable(Constants.DRIVER_LIMELIGHT);
    NetworkTableEntry txp = chair.getEntry("txp");
    NetworkTableEntry typ = chair.getEntry("typ");

    double txPixels = txp.getDouble(0.0);
    double tyPixels = typ.getDouble(0.0);

    double yDistanceMeters = -camMountHeightMeters / Math.tan(tyPixels * verticalAnglePerPixel - (camHorizontalPOVRad/2) - camMountAngleRad);
    double rayToGround = Math.sqrt(Math.pow(camMountHeightMeters, 2) + Math.pow(yDistanceMeters, 2));
    double xDistanceMeters = rayToGround * Math.tan(txPixels * horizontalAnglePerPixel - (camHorizontalPOVRad / 2));

    Logger.recordOutput("Note Align", xDistanceMeters);
    Logger.recordOutput("Note Align", yDistanceMeters);

    Translation2d robotTranslation = drive.getPose().getTranslation();
    Translation2d noteTranslation = robotTranslation.plus(new Translation2d(xDistanceMeters + 0.25, yDistanceMeters));

    Pose2d notePose = new Pose2d(noteTranslation, drive.getRotation());

    pathfindCommand = AutoBuilder.pathfindToPose(
      notePose, 
      new PathConstraints(1.5, 2.0, 540, 540));

    if (chair.getEntry("tv").getDouble(0.0) != 1.0) {
      return;
    }

    intake.setIntakeSpeed(Constants.INTAKE_SPEED);
    indexer.setIndexerSpeed(Constants.INDEXER_FEED_SPEED);

    pathfindCommand.schedule();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Leds.getInstance().autoNoteAlign = false;
    LimelightHelpers.setLEDMode_ForceOff(Constants.DRIVER_LIMELIGHT);

    intake.stopIntake();
    indexer.stopIndexer();
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathfindCommand.isFinished();
  }
}
