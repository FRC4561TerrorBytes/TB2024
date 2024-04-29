// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;

/** Add your docs here. */
public class VisionIOLimelight implements VisionIO {

    public void updateInputs(VisionIOInputs inputs) {
        LimelightHelpers.PoseEstimate mt2Pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.VISION_LIMELIGHT);

        inputs.mt2Pose = mt2Pose.pose;
        inputs.mt2Timestamp = mt2Pose.timestampSeconds;
        inputs.mt2TagCount = mt2Pose.tagCount;
        inputs.mt2AvgDistance = mt2Pose.avgTagDist;
    } 
}
