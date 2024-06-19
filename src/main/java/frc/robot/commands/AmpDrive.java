// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;

public class AmpDrive extends Command {

  private Drive drive;
  private Command pathCommand;

  public AmpDrive(Drive drive) {
    this.drive = drive;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d ampPose = new Pose2d(14.75, 7.70, Rotation2d.fromDegrees(270));

    AllianceFlipUtil.apply(ampPose);

    pathCommand = AutoBuilder.pathfindToPose(
      ampPose, 
      new PathConstraints(2, 2, 540, 540));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pathCommand.schedule();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pathCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathCommand.isFinished();
  }
}
