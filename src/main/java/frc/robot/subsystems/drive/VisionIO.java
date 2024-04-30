// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

/** Add your docs here. */
public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public Pose2d mt2Pose = new Pose2d();
        public double mt2Timestamp = 0.0;
        public int mt2TagCount = 0;
        public double mt2AvgDistance = 0.0;
        public Pose3d[] seenTags = {};
    }

    public default void updateInputs(VisionIOInputs inputs) {}
}
