// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.subsystems.Leds;
import frc.robot.util.Alert;
import frc.robot.util.AlertHandler;
import frc.robot.util.Alert.AlertType;

/** Add your docs here. */
public class IndexerIOReal implements IndexerIO {

    private final CANSparkMax m_indexer = new CANSparkMax(Constants.INDEXER, MotorType.kBrushless);
    private final DigitalInput m_rightLimitSwitch = new DigitalInput(1);
    private final DigitalInput m_leftLimitSwitch = new DigitalInput(3);

    private Alert indexerMotorDisconnectAlert;
    private Alert indexerMotorCurrentAlert; 

    public IndexerIOReal(){
        m_indexer.restoreFactoryDefaults();
        //set inverted here
        m_indexer.setSmartCurrentLimit(60, 25);
        m_indexer.setIdleMode(IdleMode.kBrake);
        
        m_indexer.burnFlash();

        indexerMotorDisconnectAlert = new Alert("Indexer Alert", "Indexer motor is not present on CAN", AlertType.ERROR);
        indexerMotorCurrentAlert = new Alert("Indexer Alert", "Indexer motor has motor/overcurrent fault", AlertType.WARNING);    }

     public void updateInputs(IndexerIOInputs inputs) {
        inputs.indexerAppliedVolts = m_indexer.getAppliedOutput();
        inputs.indexerState = !m_rightLimitSwitch.get() || !m_leftLimitSwitch.get();
        inputs.indexerCurrentAmps = m_indexer.getOutputCurrent();

        Leds.getInstance().noteInIndexer = !m_rightLimitSwitch.get() || !m_leftLimitSwitch.get();
        Logger.recordOutput("LeftLimit",!m_leftLimitSwitch.get());
        Logger.recordOutput("RightLimit",!m_rightLimitSwitch.get());

        // AlertHandler.reportSparkMaxFault("Indexer Alert", m_indexer, indexerMotorDisconnectAlert, indexerMotorCurrentAlert);
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

    @Override
    public boolean getDisconnect(){
        return indexerMotorDisconnectAlert.getState();
    }
}