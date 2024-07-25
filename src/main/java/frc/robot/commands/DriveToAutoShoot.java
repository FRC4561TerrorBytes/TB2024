// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.AllianceFlipUtil;

public class DriveToAutoShoot extends Command {

  private Command pathCommand;

  public DriveToAutoShoot() {}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d shootingPose = new Pose2d(4.5, 5.5, new Rotation2d());

    shootingPose = AllianceFlipUtil.apply(shootingPose);

    pathCommand = AutoBuilder.pathfindToPose(
      shootingPose,
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
