// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IndexerIO {

    @AutoLog
    public static class IndexerIOInputs {
        public double indexerAppliedVolts = 0.0;
        public boolean indexerState = false;
        public double[] indexerCurrentAmps = new double[] {};
    }

    public default boolean getIndexerState(){
        return false;
    };

    public default void updateInputs(IndexerIOInputs inputs) {};

    public default void setIndexerSpeed(double speed) {};

    public default void stopIndexer() {};
}