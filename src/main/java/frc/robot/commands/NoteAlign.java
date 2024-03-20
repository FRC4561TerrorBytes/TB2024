// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Detector;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;

public class NoteAlign extends Command {

  private Drive drive;
  private Indexer indexer;
  private Intake intake;

  /** Creates a new NoteAlign. */
  public NoteAlign(Drive drive, Indexer indexer, Intake intake) {
    this.drive = drive;
    this.indexer = indexer;
    this.intake = intake;

    addRequirements(drive, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LimelightHelpers.setPipelineIndex("limelight-driver", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LimelightResults results = LimelightHelpers.getLatestResults("limelight-driver");

    LimelightTarget_Detector[] notes = results.targetingResults.targets_Detector;
    LimelightTarget_Detector closestNote = getClosestNote(notes);

    Logger.recordOutput("NoteAlign/Note Count", notes.length);

    double xRequest;
    double yRequest;

    boolean inXTol = false;
    boolean inYTol = false;

    if (closestNote == null) {
      System.out.println("\n\n note null :(\n\n");
      return;
    }

    if (closestNote.ty > -25) {
      yRequest = 0.2;
    } else {
      yRequest = 0.0;
      inYTol = true;
    }

    Logger.recordOutput("NoteAlign/yRequest", yRequest);
    Logger.recordOutput("NoteAlign/inYTol", inYTol);

    if (closestNote.tx < -1) {
      xRequest = 0.2;
    } else if (closestNote.tx > 1) {
      xRequest = -0.2;
    } else {
      xRequest = 0.0;
      inXTol = true;
    }

    Logger.recordOutput("NoteAlign/xRequest", xRequest);
    Logger.recordOutput("NoteAlign/inXTol", inXTol);

    drive.runVelocity(new ChassisSpeeds(yRequest, xRequest, 0.0));

    if (inXTol && inYTol) {
      intake.setIntakeSpeed(Constants.INTAKE_SPEED);
      indexer.setIndexerSpeed(Constants.INDEXER_FEED_SPEED);
      drive.runVelocity(new ChassisSpeeds(0.2, 0, 0));
    }
  }

  private LimelightTarget_Detector getClosestNote(LimelightTarget_Detector[] noteArray) {
    LimelightTarget_Detector closestNote = null;
    double closest = -999;

    for (LimelightTarget_Detector note : noteArray) {
      double distance = note.ty;
      if (distance > closest) {
        closestNote = note;
        closest = note.ty;
      }
    }
    return closestNote;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
    indexer.stopIndexer();
    intake.stopIntake();
    LimelightHelpers.setPipelineIndex("limelight-driver", 1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return indexer.noteInIndexer();
  }
}
