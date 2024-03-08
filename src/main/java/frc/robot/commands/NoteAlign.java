// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Detector;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;

public class NoteAlign extends Command {

  private Drive drive;
  private Indexer indexer;
  private Intake intake;
  private Arm arm;

  /** Creates a new NoteAlign. */
  public NoteAlign(Drive drive, Indexer indexer, Intake intake, Arm arm) {
    this.drive = drive;
    this.indexer = indexer;
    this.intake = intake;
    this.arm = arm;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LimelightHelpers.setPipelineIndex("limelight-test", 1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LimelightResults results = LimelightHelpers.getLatestResults("limelight-test");

    LimelightTarget_Detector[] notes = results.targetingResults.targets_Detector;
    LimelightTarget_Detector closestNote = getClosestNote(notes);

    double xRequest;
    double yRequest;

    boolean inXTol = false;
    boolean inYTol = false;

    if (closestNote.tx < -0.1) {
      xRequest = 0.2;
    } else {
      xRequest = 0.0;
      inXTol = true;
    }

    if (closestNote.ty < -0.1) {
      yRequest = 0.2;
    } else if (closestNote.ty > 0.1) {
      yRequest = -0.2;
    } else {
      yRequest = 0.0;
      inYTol = true;
    }

    DriveCommands.joystickDrive(drive, () -> xRequest, () -> yRequest, () -> 0.0);

    if (inXTol && inYTol) {
      new IntakeCommand(intake, indexer, arm, new GenericHID(0));
      DriveCommands.joystickDrive(drive, () -> 0.2, () -> 0.0, () -> 0.0);
    }
  }

  private LimelightTarget_Detector getClosestNote(LimelightTarget_Detector[] noteArray) {
    LimelightTarget_Detector closestNote = null;

    for (LimelightTarget_Detector note : noteArray) {
      double distance = note.tx;
      if (distance < closestNote.tx) {
        closestNote = note;
      }
    }
    return closestNote;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
    LimelightHelpers.setPipelineIndex("limelight-test", 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return indexer.noteInIndexer();
  }
}
