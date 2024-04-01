// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotContainer.shootPositions;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class ShootCommand extends Command {
  /** Creates a new ShootCommand. */
  Shooter shooter;
  Intake intake;
  Indexer indexer;
  Arm arm;
  double targetMPS = 0;
  shootPositions shooterPositions;

  public ShootCommand(Shooter shooter, Indexer indexer, Intake intake, Arm arm, shootPositions shooterPosition) {
    this.shooter = shooter;
    this.indexer = indexer;
    this.intake = intake;
    this.arm = arm;
    this.shooterPositions = shooterPosition;

    addRequirements(shooter, indexer, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetMPS = shooter.getVelocity();
    shooter.setFlywheelSpeed(shooterPositions.getShootSpeed());
    // arm.setArmSetpoint(shooter.getPivotAngle());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooter.flywheelUpToSpeed(shooterPositions.getShootSpeed() * 0.875)){
      indexer.setIndexerSpeed(Constants.INDEXER_FEED_SPEED);
      intake.setIntakeSpeed(0.5);
      if (Constants.currentMode == Mode.SIM) {
        shooter.launchCommand().withTimeout(0.5).schedule();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setFlywheelSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return !indexer.noteInIndexer();//Should be beam breaks when we get them
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
