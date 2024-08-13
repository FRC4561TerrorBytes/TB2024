// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotContainer.shootPositions;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.AllianceFlipUtil;

public class AutoShootCommand extends Command {

  private final Arm arm;
  private final Shooter shooter;
  private final Indexer indexer;
  private final Intake intake;
  private final Drive drive;
  private double targetMPS;

  private static final Translation3d blueSpeaker = new Translation3d(0.225, 5.55, 2.1);

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
    Translation2d speakerAlliance = AllianceFlipUtil.apply(blueSpeaker).toTranslation2d();
    Pose2d robotRotation = drive.getPose();

    Transform2d pose = robotRotation.minus(new Pose2d(speakerAlliance, Rotation2d.fromDegrees(180.0)));

    Logger.recordOutput("Auto Shoot/Speaker Rot", pose);

    double txAngle = 0.0;

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
    if (Constants.currentMode == Mode.SIM) {
      return false;
    }
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
