// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer.shootPositions;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;

public class LobShootCommand extends Command {

  private final Arm arm;
  private final Shooter shooter;
  private final Indexer indexer;

  public LobShootCommand(Arm arm, Shooter shooter, Indexer indexer) {
    this.arm = arm;
    this.shooter = shooter;
    this.indexer = indexer;

    addRequirements(arm, shooter, indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setArmSetpoint(shootPositions.LOB.getShootAngle());
    shooter.setFlywheelSpeed(15.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter.flywheelUpToSpeed(15.0 * 0.875) && arm.armAtSetpoint()) {
      indexer.setIndexerSpeed(Constants.INDEXER_FEED_SPEED);
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
    if(!indexer.noteInIndexer()){
      for(int i = 0; i < 25; i++){
        continue;
      }
      return true;
    }
    else{
      return false;
    }
  }
}
