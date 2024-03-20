// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

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
    PathPoint ampPoint = new PathPoint(new Translation2d());
    PathPoint currentPoint = new PathPoint(drive.getPose().getTranslation());
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      ampPoint = new PathPoint(new Translation2d(1.80, 7.70));
    } else {
      ampPoint = new PathPoint(new Translation2d(14.75, 7.70));
    }

    ArrayList<PathPoint> points = new ArrayList<PathPoint>();
    points.add(currentPoint);
    points.add(ampPoint);

    PathPlannerPath path = PathPlannerPath.fromPathPoints(
      points,
      new PathConstraints(1, 1, 540, 720),
      new GoalEndState(0, Rotation2d.fromDegrees(0)));

      pathCommand = AutoBuilder.followPath(path);

      pathCommand.schedule();
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathCommand.isFinished();
  }
}
