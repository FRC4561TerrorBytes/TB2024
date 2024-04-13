// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.subsystems.Leds;

/** Add your docs here. */
public class IndexerIOReal implements IndexerIO {

    private final CANSparkMax m_indexer = new CANSparkMax(Constants.INDEXER, MotorType.kBrushless);
    private final DigitalInput m_rightLimitSwitch = new DigitalInput(1);
    private final DigitalInput m_leftLimitSwitch = new DigitalInput(3);

    public IndexerIOReal(){
        m_indexer.restoreFactoryDefaults();
        //set inverted here
        m_indexer.setSmartCurrentLimit(60, 25);
        m_indexer.setIdleMode(IdleMode.kBrake);
        
        m_indexer.burnFlash();
    }

     public void updateInputs(IndexerIOInputs inputs) {
        inputs.indexerAppliedVolts = m_indexer.getAppliedOutput();
        inputs.indexerState = !m_rightLimitSwitch.get() || !m_leftLimitSwitch.get();
        inputs.indexerCurrentAmps = m_indexer.getOutputCurrent();

        Leds.getInstance().noteInIndexer = !m_rightLimitSwitch.get() || !m_leftLimitSwitch.get();

        // SmartDashboard.putNumber("Indexer Current", m_indexer.getOutputCurrent());
    }

    public void setIndexerSpeed(double speed){
        m_indexer.set(speed);
    }

    public void stopIndexer(){
        setIndexerSpeed(0);
    }

    public boolean getIndexerState(){
        return !m_rightLimitSwitch.get() || !m_leftLimitSwitch.get();
    }
}