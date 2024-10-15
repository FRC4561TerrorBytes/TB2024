// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

/** Add your docs here. */
public class IndexerIOSim implements IndexerIO {
    public static final double LOOP_PERIOD_SECS = 0.02;

    private double indexerAppliedVolts = 0.0;

    private DCMotorSim indexerMotorSim = new DCMotorSim(DCMotor.getNeo550(1), Constants.INDEXER_MOTOR_GEAR_RATIO, 0.025);

    public void updateInputs(IndexerIOInputs inputs) {
        indexerMotorSim.update(LOOP_PERIOD_SECS);
        inputs.indexerAppliedVolts = indexerAppliedVolts;
    }


    public void setIndexerSpeed(double speed){
        indexerAppliedVolts = speed / 12;
        indexerMotorSim.setInputVoltage(speed / 12);
    }

    public void stopIndexer(){
        indexerMotorSim.setInputVoltage(0);
    }

}