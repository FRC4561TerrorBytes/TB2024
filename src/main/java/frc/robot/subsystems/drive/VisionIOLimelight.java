// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;

/** Add your docs here. */
public class VisionIOLimelight implements VisionIO {

    private AprilTagFieldLayout aprilTagMap = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

    public void updateInputs(VisionIOInputs inputs) {
        LimelightHelpers.PoseEstimate mt2Pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.VISION_LIMELIGHT);

        inputs.mt2Pose = mt2Pose.pose;
        inputs.mt2Timestamp = mt2Pose.timestampSeconds;
        inputs.mt2TagCount = mt2Pose.tagCount;
        inputs.mt2AvgDistance = mt2Pose.avgTagDist;

        Pose3d[] tagPoses = {};
        int i = 0;
        for (RawFiducial tag : mt2Pose.rawFiducials) {
            tagPoses[i] = aprilTagMap.getTagPose(tag.id).get();
            i++;
        }

        inputs.seenTags = tagPoses;
    } 
}
