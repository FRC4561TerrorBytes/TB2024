// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer.shootPositions;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class AutoShootCommand extends Command {

  private final Arm arm;
  private final Shooter shooter;
  private final Indexer indexer;
  private final Intake intake;
  private final Drive drive;
  private double targetMPS;


  public AutoShootCommand(Arm arm, Shooter shooter, Indexer indexer, Intake intake, Drive drive) {
    this.arm = arm;
    this.shooter = shooter;
    this.indexer = indexer;
    this.intake = intake;
    this.drive = drive;

    addRequirements(shooter, indexer, intake, drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Leds.getInstance().autoShootCommand = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double armAngleInterpolated = shooter.interpolateArmAngle(drive.getDistanceFromSpeaker());
    arm.setArmSetpoint(armAngleInterpolated);
    targetMPS = 25;

    shooter.setFlywheelSpeed(targetMPS);

    if (shooter.flywheelUpToSpeed(targetMPS) && arm.armAtSetpoint()) {
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
    if(!interrupted){
      arm.setArmSetpoint(shootPositions.STOW.getShootAngle());
    }
    Leds.getInstance().autoShootCommand = false;
    Logger.recordOutput("Auto Rotate/Rotating", false);
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
