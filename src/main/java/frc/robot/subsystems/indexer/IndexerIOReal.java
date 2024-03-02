// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;

import frc.robot.Constants;

/** Add your docs here. */
public class IndexerIOReal implements IndexerIO {

    private final CANSparkMax m_indexer = new CANSparkMax(Constants.INDEXER, MotorType.kBrushless);
    private final SparkLimitSwitch m_limitSwitch = m_indexer.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

    public IndexerIOReal(){
        m_indexer.restoreFactoryDefaults();
        //set inverted here
        m_indexer.setSmartCurrentLimit(40);
        m_indexer.setIdleMode(IdleMode.kBrake);
        
        m_indexer.burnFlash();
        m_limitSwitch.enableLimitSwitch(false);
    }

     public void updateInputs(IndexerIOInputs inputs) {
        inputs.indexerAppliedVolts = m_indexer.getAppliedOutput();
        inputs.indexerState = m_limitSwitch.isPressed();
        inputs.indexerCurrentAmps = m_indexer.getOutputCurrent();

        // SmartDashboard.putNumber("Indexer Current", m_indexer.getOutputCurrent());
    }

    public void setIndexerSpeed(double speed){
        m_indexer.set(speed);
    }

    public void stopIndexer(){
        setIndexerSpeed(0);
    }

    public boolean getIndexerState(){
        return m_limitSwitch.isPressed();
    }
}