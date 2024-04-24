// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
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
    //drive.stop();
    NetworkTable chair = NetworkTableInstance.getDefault().getTable(Constants.VISION_LIMELIGHT);
    NetworkTableEntry tx = chair.getEntry("tx");
    double txAngle = tx.getDouble(0.0);

    Leds.getInstance().autoShootCommand = true;
    Leds.getInstance().autoShootStartAngle = txAngle;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    NetworkTable chair = NetworkTableInstance.getDefault().getTable(Constants.VISION_LIMELIGHT);
    NetworkTableEntry tx = chair.getEntry("tx");
    double txAngle = tx.getDouble(0.0);

    // if (chair.getEntry("tid").getDouble(0.0) == 0.0) {
    //   return;
    // }

    if (txAngle > 3.5) {
      drive.runVelocity(new ChassisSpeeds(0, 0, -Units.degreesToRadians(20)));
      Logger.recordOutput("Auto Rotate/Rotating", true);
    } else if (txAngle < -3.5) {
      drive.runVelocity(new ChassisSpeeds(0, 0, Units.degreesToRadians(20)));
      Logger.recordOutput("Auto Rotate/Rotating", true);
    } else {
      Logger.recordOutput("Auto Rotate/Rotating", false);
      drive.stopWithX();
    }

    double distanceMeters = Units.inchesToMeters(shooter.findFlatDistanceWithVision());
    double armAngleInterpolated = shooter.interpolateArmAngle(distanceMeters);
    arm.setArmSetpoint(armAngleInterpolated);
    targetMPS = 25;

    if (txAngle < 3.5 && txAngle > -3.5) {
      shooter.setFlywheelSpeed(targetMPS);

      if (shooter.flywheelUpToSpeed(targetMPS * 0.875) && arm.armAtSetpoint()) {
        indexer.setIndexerSpeed(Constants.INDEXER_FEED_SPEED);
        intake.setIntakeSpeed(0.5);
      }
    }

    Logger.recordOutput("Auto Rotate/TX", txAngle);
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
