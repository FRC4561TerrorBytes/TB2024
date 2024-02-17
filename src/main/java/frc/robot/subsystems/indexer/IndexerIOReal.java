// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import com.revrobotics.CANSparkMax;

import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

/** Add your docs here. */
public class IndexerIOReal implements IndexerIO {

    private final CANSparkMax m_indexer = new CANSparkMax(Constants.INDEXER, MotorType.kBrushless);
    
    public IndexerIOReal(){
    m_indexer.restoreFactoryDefaults();
    //set inverted here
    m_indexer.setSmartCurrentLimit(20);
    m_indexer.setIdleMode(IdleMode.kBrake);
    m_indexer.burnFlash();
    }

     public void updateInputs(ShooterIOInputs inputs) {
        inputs.indexerAppliedVolts = m_indexer.getAppliedOutput();
    }

    public void setIndexerSpeed(double speed){
        m_indexer.set(speed);
    }

    public void stopIndexer(){
        setIndexerSpeed(0);
    }
}
