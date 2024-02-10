// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants;

/** Add your docs here. */
public class ShooterIOReal implements ShooterIO {

    private TalonFX m_leftFlywheel = new TalonFX(Constants.LEFT_FLYWHEEL);
    private TalonFX m_rightFlywheel = new TalonFX(Constants.RIGHT_FLYWHEEL);
    private CANSparkMax m_indexer = new CANSparkMax(Constants.INDEXER, MotorType.kBrushless);

    public ShooterIOReal() {
        //constructor go brrrrrrr
        var leftConfig = new TalonFXConfiguration();
        //set inverted here
        leftConfig.Feedback.SensorToMechanismRatio = Constants.FLYWHEEL_CIRCUMFERENCE/60;
        leftConfig.CurrentLimits.SupplyCurrentLimit = 40;
        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        var leftSlot0Config = leftConfig.Slot0;
        leftSlot0Config.kS = 0.25; // Add 0.25 V output to overcome static friction
        leftSlot0Config.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        leftSlot0Config.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        leftSlot0Config.kP = 0.11; // An error of 1 rps results in 0.11 V output
        leftSlot0Config.kI = 0; // no output for integrated error
        leftSlot0Config.kD = 0; // no output for error derivative

        var leftMotionMagicConfig = leftConfig.MotionMagic;
        leftMotionMagicConfig.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
        leftMotionMagicConfig.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)

        m_leftFlywheel.getConfigurator().apply(leftConfig);

        var rightConfig = new TalonFXConfiguration();
        //set inverted here
        rightConfig.Feedback.SensorToMechanismRatio = Constants.FLYWHEEL_CIRCUMFERENCE/60;
        rightConfig.CurrentLimits.SupplyCurrentLimit = 40;
        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        var rightSlot0Config = leftConfig.Slot0;
        rightSlot0Config.kS = 0.25; // Add 0.25 V output to overcome static friction
        rightSlot0Config.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        rightSlot0Config.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        rightSlot0Config.kP = 0.11; // An error of 1 rps results in 0.11 V output
        rightSlot0Config.kI = 0; // no output for integrated error
        rightSlot0Config.kD = 0; // no output for error derivative

        var rightMotionMagicConfig = leftConfig.MotionMagic;
        rightMotionMagicConfig.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
        rightMotionMagicConfig.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)

        m_rightFlywheel.getConfigurator().apply(rightConfig);

        m_indexer.restoreFactoryDefaults();
        //set inverted here
        m_indexer.setSmartCurrentLimit(20);
        m_indexer.setIdleMode(IdleMode.kBrake);
        m_indexer.burnFlash();
    }

    public void updateInputs(ShooterIOInputs inputs) {
        inputs.indexerAppliedVolts = m_indexer.getAppliedOutput();
        inputs.shooterVelocityMPS = m_leftFlywheel.getVelocity().getValueAsDouble();
        inputs.shooterCurrentAmps = new double[] {m_leftFlywheel.getSupplyCurrent().getValueAsDouble()};
    }

    public void setFlywheelSpeed(double velocity){
        final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);
        m_rightFlywheel.setControl(m_request.withVelocity(velocity));
        m_leftFlywheel.setControl(m_request.withVelocity(velocity));
    }
    public void stopFlywheel(){
        final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);
        m_rightFlywheel.setControl(m_request.withVelocity(0));
        m_leftFlywheel.setControl(m_request.withVelocity(0));
    }

    public void setIndexerSpeed(double speed){
        m_indexer.set(speed);
    }

    public void stopIndexer(){
        m_indexer.set(0);
    }
}
