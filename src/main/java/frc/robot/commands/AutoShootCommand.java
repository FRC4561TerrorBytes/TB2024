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

public class AutoShootCommand extends Command {

  private final Drive drive;
  private final Arm arm;
  private final Shooter shooter;
  private final Indexer indexer;
  private final Intake intake;
  private double targetMPS;

  public AutoShootCommand(Drive drive, Arm arm, Shooter shooter, Indexer indexer, Intake intake) {
    this.drive = drive;
    this.arm = arm;
    this.shooter = shooter;
    this.indexer = indexer;
    this.intake = intake;

    addRequirements(drive, shooter, indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Dont think this will actually work but ;-;
    new FaceSpeaker(drive);
    
    arm.setArmSetpoint(shooter.calculateArmRotations());
    targetMPS = 40;
    shooter.setFlywheelSpeed(targetMPS);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter.flywheelUpToSpeed(targetMPS * 0.875) && arm.armAtSetpoint()) {
      indexer.setIndexerSpeed(Constants.INDEXER_FEED_SPEED);
      intake.setIntakeSpeed(0.5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopFlywheel();
    indexer.stopIndexer();
    intake.stopIntake();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!indexer.noteInIndexer()) {
      for (int i = 0; i < 25; i++) {
        continue;
      }
      return true;
    } else {
      return false;
    }
  }
}
