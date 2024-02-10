// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.NoteVisualizer;

public class ShootCommandIO extends Command {
  /** Creates a new ShootCommand. */
  Shooter shooter;
  Drive m_driveSubsystem;
  NoteVisualizer visualizer;
  double targetMPS = 0;

  public ShootCommandIO(Shooter shooter, Drive driveSubsystem, NoteVisualizer visualizer) {
    this.shooter = shooter;
    m_driveSubsystem = driveSubsystem;
    this.visualizer = visualizer;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.calculateShooter(m_driveSubsystem.getDistanceFromSpeaker());
    targetMPS = shooter.getVelocity();
    shooter.setFlywheelSpeed(targetMPS);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooter.flywheelUpToSpeed(targetMPS)){
      System.out.println("\n\n\n\ndiuwabdwa\n\n\n\n");
      shooter.setIndexerSpeed(Constants.INDEXER_FEED_SPEED);
      shooter.launchCommand().schedule();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopFlywheel();
    shooter.stopIndexer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.flywheelUpToSpeed(targetMPS);
  }
}
