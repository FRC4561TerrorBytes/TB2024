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
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer.shootPositions;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;

public class AutoNoteAlignCommand extends Command {

  private final Drive drive;
  private final Intake intake;
  private final Indexer indexer;
  private final Arm arm;

  double camMountHeightIn = 20;
  double camMountAngleDeg = -22.5;

  double deadzone = 10;
  double rotDeg = 20;

  public AutoNoteAlignCommand(Drive drive, Intake intake, Indexer indexer, Arm arm) {
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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    NetworkTable chair = NetworkTableInstance.getDefault().getTable(Constants.DRIVER_LIMELIGHT);

    if (chair.getEntry("tv").getDouble(0.0) != 1.0) {return;}

    NetworkTableEntry tx = chair.getEntry("tx");
    double txDeg = tx.getDouble(0.0);

    if (txDeg > deadzone) {
      drive.runVelocity(new ChassisSpeeds(0, 0, -Units.degreesToRadians(rotDeg)));
    } else if (txDeg < -deadzone) {
      drive.runVelocity(new ChassisSpeeds(0, 0, Units.degreesToRadians(rotDeg)));
    }

    NetworkTableEntry ty = chair.getEntry("ty");
    double tyDeg = ty.getDouble(0.0);

    double angleToGoalRad = Units.degreesToRadians(camMountAngleDeg + tyDeg);

    double distanceIn = (0 - camMountHeightIn) / Math.tan(angleToGoalRad);

    Logger.recordOutput("Note Align/Angle to note", angleToGoalRad);
    Logger.recordOutput("Note Align/Distance", distanceIn);

    intake.setIntakeSpeed(Constants.INTAKE_SPEED);
    indexer.setIndexerSpeed(Constants.INDEXER_FEED_SPEED);

    drive.runVelocity(new ChassisSpeeds(1.5 * (distanceIn / 90), 0, 0));
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
