// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoNoteAlignSequential extends SequentialCommandGroup {

  private Drive drive;
  private Intake intake;
  private Indexer indexer;
  private Arm arm;

  public AutoNoteAlignSequential(Drive drive, Intake intake, Indexer indexer, Arm arm) {
    this.drive = drive;
    this.intake = intake;
    this.indexer = indexer;
    this.arm = arm;

    addCommands(new AutoNoteAlignCommand(drive, intake, indexer).andThen(new IntakeCommand(intake, indexer, arm)));
  }
}
