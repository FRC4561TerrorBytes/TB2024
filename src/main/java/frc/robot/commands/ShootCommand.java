// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;

public class ShootCommand extends Command {
  /** Creates a new ShootCommand. */
  Shooter shooter;
  Drive m_driveSubsystem;
  private Indexer indexer;
  //Arm arm;
  double targetMPS = 0;

  public ShootCommand(Shooter shooter, Drive driveSubsystem, Indexer indexer) {
    this.shooter = shooter;
    m_driveSubsystem = driveSubsystem;
    this.indexer = indexer;
    //this.arm = arm;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.calculateShooter(m_driveSubsystem.getDistanceFromSpeaker());
    targetMPS = shooter.getVelocity();
    shooter.setFlywheelSpeed(targetMPS);
    //arm.setArmSetpoint(shooter.getPivotAngle());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooter.flywheelUpToSpeed(targetMPS)){
      indexer.setIndexerSpeed(1);
      shooter.launchCommand().withTimeout(0.5).schedule();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopFlywheel();
    indexer.stopIndexer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.flywheelUpToSpeed(targetMPS); //Should be beam breaks when we get them
  }
}
