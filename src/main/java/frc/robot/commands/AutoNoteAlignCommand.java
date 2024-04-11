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
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;

public class AutoNoteAlignCommand extends Command {

  private final Drive drive;
  private final Intake intake;
  private final Indexer indexer;

  public AutoNoteAlignCommand(Drive drive, Intake intake, Indexer indexer) {
    this.drive = drive;
    this.intake = intake;
    this.indexer = indexer;

    addRequirements(intake, indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Leds.getInstance().autoNoteAlign = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    NetworkTable chair = NetworkTableInstance.getDefault().getTable(Constants.DRIVER_LIMELIGHT);
    NetworkTableEntry tx = chair.getEntry("tx");
    double txAngle = tx.getDouble(0.0);

    if (txAngle > 3) {
      drive.runVelocity(new ChassisSpeeds(0, 0, -Units.degreesToRadians(20)));
      Logger.recordOutput("Note ALign/Rotating", true);
    } else if (txAngle < -3) {
      drive.runVelocity(new ChassisSpeeds(0, 0, Units.degreesToRadians(20)));
      Logger.recordOutput("Note ALign/Rotating", true);
    } else {
      Logger.recordOutput("Note ALign/Rotating", false);
      drive.stop();
    }

    if (txAngle < 3.5 && txAngle > -3.5) {
      intake.setIntakeSpeed(Constants.INTAKE_SPEED);
      indexer.setIndexerSpeed(Constants.INDEXER_FEED_SPEED);
      drive.runVelocity(new ChassisSpeeds(-0.5, 0, 0));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Leds.getInstance().autoNoteAlign = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.getIntakeBreak();
  }
}
