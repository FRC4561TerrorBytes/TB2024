package frc.robot.subsystems.drive;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;

public class PoseEstimatorSubsystem {
    private Drive m_driveSubsystem;
    private final AprilTagFieldLayout m_aprilTagFieldLayout;


    public PoseEstimatorSubsystem(Drive driveSubsystem){
        m_driveSubsystem = driveSubsystem;

         AprilTagFieldLayout layout;
        try {
            layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            var alliance = DriverStation.getAlliance();
            layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        } 
        catch(IOException e) {
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            layout = null;
        }
        m_aprilTagFieldLayout = layout;
    }
}
