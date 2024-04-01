// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class AutoShootCommand extends Command {

  private final Arm arm;
  private final Shooter shooter;
  private final Indexer indexer;
  private final Intake intake;
  private double targetMPS;

  public AutoShootCommand(Arm arm, Shooter shooter, Indexer indexer, Intake intake) {
    this.arm = arm;
    this.shooter = shooter;
    this.indexer = indexer;
    this.intake = intake;

    addRequirements(shooter, indexer, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {    
    double distanceMeters = Units.inchesToMeters(shooter.findFlatDistanceWithVision());
    double armAngleInterpolated = shooter.interpolateArmAngle(distanceMeters);
    arm.setArmSetpoint(armAngleInterpolated);
    targetMPS = 25;
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
