// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.Drive;

public class ShootCommand extends Command {
  /** Creates a new ShootCommand. */
  ShooterSubsystem m_shooterSubsystem;
  Drive m_driveSubsystem;
  double targetMPS = 0;

  public ShootCommand(ShooterSubsystem shooterSubsystem, Drive driveSubsystem) {
    m_shooterSubsystem = shooterSubsystem;
    m_driveSubsystem = driveSubsystem;
    addRequirements(m_shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterSubsystem.calculateShooter(m_driveSubsystem.getDistanceFromSpeaker());
    targetMPS = m_shooterSubsystem.getVelocity();
    m_shooterSubsystem.setFlywheelSpeed(targetMPS);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_shooterSubsystem.flywheelUpToSpeed(targetMPS)){
      m_shooterSubsystem.setIndexerSpeed(Constants.INDEXER_FEED_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.stopFlywheel();
    m_shooterSubsystem.stopIndexer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_shooterSubsystem.noteShot();
  }
}
