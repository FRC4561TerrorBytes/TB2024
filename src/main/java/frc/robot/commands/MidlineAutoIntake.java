// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer.shootPositions;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.AllianceFlipUtil;

public class MidlineAutoIntake extends Command {

  private final Drive drive;
  private final Intake intake;
  private final Indexer indexer;
  private final Arm arm;

  double camMountHeightIn = 20;
  double camMountAngleDeg = -22.5;

  // TODO: refine deadzone
  double deadzone = 20;

  boolean inRotTol;

  public MidlineAutoIntake(Drive drive, Intake intake, Indexer indexer, Arm arm) {
    this.drive = drive;
    this.intake = intake;
    this.indexer = indexer;
    this.arm = arm;

    addRequirements(intake, indexer, drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.stop();
    LimelightHelpers.setLEDMode_ForceOn(Constants.DRIVER_LIMELIGHT);
    arm.setArmSetpoint(shootPositions.STOW.getShootAngle());
    Leds.getInstance().autoNoteAlign = true;
    inRotTol = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    NetworkTable chair = NetworkTableInstance.getDefault().getTable(Constants.DRIVER_LIMELIGHT);

    NetworkTableEntry tx = chair.getEntry("tx");
    double txDeg = tx.getDouble(-1);

    NetworkTableEntry ty = chair.getEntry("ty");
    double tyDeg = ty.getDouble(-1);

    if (chair.getEntry("tv").getDouble(0.0) != 0.0 || txDeg == -1 || tyDeg == -1) {
      Transform2d centerNote = new Transform2d(8.28, 4.10, Rotation2d.fromDegrees(0.0));
      Rotation2d rotToCenter = new Rotation2d();

      rotToCenter = new Rotation2d(
        centerNote.getX() - drive.getPose().getX(),
        centerNote.getY() - drive.getPose().getY());

      Logger.recordOutput("Note Align/Rotation to Center", new Pose2d(drive.getPose().getTranslation(), rotToCenter));

      drive.runVelocity(new ChassisSpeeds(0, 0, -0.25 * drive.getMaxAngularSpeedRadPerSec()));
    }

    double angleToGoalRad = Units.degreesToRadians(camMountAngleDeg + tyDeg);

    double distanceIn = (0 - camMountHeightIn) / Math.tan(angleToGoalRad);

    Logger.recordOutput("Note Align/Distance", distanceIn);

    double adjustedDeadzone = deadzone / distanceIn;

    Logger.recordOutput("Note Align/Adjusted Deadzone", adjustedDeadzone);

    if (Math.abs(txDeg) < adjustedDeadzone) {
      inRotTol = true;
    } else {
      drive.runVelocity(new ChassisSpeeds(0, 0, -((txDeg / 225) * drive.getMaxAngularSpeedRadPerSec())));
    }

    Logger.recordOutput("Note Align/inRotTol", inRotTol);

    if(inRotTol) {
      intake.setIntakeSpeed(Constants.INTAKE_SPEED);
      indexer.setIndexerSpeed(Constants.INDEXER_FEED_SPEED);

      drive.runVelocity(new ChassisSpeeds(2.0 * (distanceIn / 40), 0, 0));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Leds.getInstance().autoNoteAlign = false;
    LimelightHelpers.setLEDMode_ForceOff(Constants.DRIVER_LIMELIGHT);

    intake.stopIntake();
    indexer.stopIndexer();
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return indexer.noteInIndexer();
  }
}
