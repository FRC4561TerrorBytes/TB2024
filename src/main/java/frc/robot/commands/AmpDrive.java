// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.util.AllianceFlipUtil;

public class AmpDrive extends Command {

  private Drive drive;
  private Indexer indexer;
  private Command pathCommand;

  public AmpDrive(Drive drive, Indexer indexer) {
    this.drive = drive;
    this.indexer = indexer;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d ampPose = new Pose2d(1.83, 7.73, Rotation2d.fromDegrees(270));

    ampPose = AllianceFlipUtil.apply(ampPose);

    pathCommand = AutoBuilder.pathfindThenFollowPath(
      PathPlannerPath.fromPathFile("Amp Path"), 
      new PathConstraints(1, 1, 360, 360));

    Leds.getInstance().ampDrive = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pathCommand.schedule();

    if (pathCommand.isFinished()) {
      indexer.setIndexerSpeed(-0.4);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pathCommand.end(interrupted);
    Leds.getInstance().ampDrive = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !indexer.noteInIndexer();
  }
}
