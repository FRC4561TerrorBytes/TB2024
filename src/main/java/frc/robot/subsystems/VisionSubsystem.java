// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.drive.Drive;

public class VisionSubsystem extends SubsystemBase {

  Drive m_driveSubsytem;

  public VisionSubsystem(Drive driveSubsystem) {
    m_driveSubsytem = driveSubsystem;
  }

  private LimelightTarget_Fiducial getClosestTag(String cameraName) {
    double closest = 100;
    LimelightTarget_Fiducial target = null;
    LimelightTarget_Fiducial[] targetList = LimelightHelpers.getLatestResults(cameraName).targetingResults.targets_Fiducials;
    for (LimelightTarget_Fiducial i : targetList) {
      double value = i.tx;
      if (value < closest) {
        closest = value;
        target = i;
      }
    }
    return target;
  }

  public LimelightTarget_Fiducial getTagByID(LimelightTarget_Fiducial[] targets, double ID) {
    for (LimelightTarget_Fiducial t : targets) {
      if (t.fiducialID == ID) {
        return t;
      }
    }
    return null;
  }

  public void updateOdometry() {
    LimelightResults results = LimelightHelpers.getLatestResults("limelight-right");
    if (getClosestTag("limelight-right") != null) {
      //m_driveSubsytem.addVision(results);
      Logger.recordOutput("updating with tags", true);
    } else {
      Logger.recordOutput("updating with tags", false);
    }
  }

  @Override
  public void periodic() {
    updateOdometry();
  }
}