// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.NoteVisualizer;

public class AmpShoot extends Command {
  /** Creates a new ShootCommand. */
  Shooter shooter;
  Drive m_driveSubsystem;
  Intake intake;
  Indexer indexer;
  Arm arm;
  NoteVisualizer visualizer;
  double targetMPS = 0;

  public AmpShoot(Shooter shooter, Drive driveSubsystem, Indexer indexer, Intake intake, Arm arm, NoteVisualizer visualizer) {
    this.shooter = shooter;
    m_driveSubsystem = driveSubsystem;
    this.indexer = indexer;
    this.intake = intake;
    this.arm = arm;
    this.visualizer = visualizer;

    addRequirements(shooter, indexer, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.calculateShooter(m_driveSubsystem.getDistanceFromSpeaker());
    targetMPS = shooter.getVelocity();
    shooter.setFlywheelSpeed(3.25);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooter.flywheelUpToSpeed(2.75)){
        indexer.setIndexerSpeed(Constants.INDEXER_FEED_SPEED);
        intake.setIntakeSpeed(0.5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // new WaitCommand(1.0).schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // return !indexer.noteInIndexer();//Should be beam breaks when we get them
    // if(!indexer.noteInIndexer()){
    //   for(int i = 0; i < 25; i++){
    //     continue;
    //   }
    //   return true;
    // }
    // else{
    //   return false;
    // }
  }
}
