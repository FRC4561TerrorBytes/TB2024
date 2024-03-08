// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

/** Add your docs here. */
public class ShooterIOReal implements ShooterIO {

    private TalonFX m_leftFlywheel = new TalonFX(Constants.LEFT_FLYWHEEL);
    private TalonFX m_rightFlywheel = new TalonFX(Constants.RIGHT_FLYWHEEL);

    private final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);

    public ShooterIOReal() {
        //constructor go brrrrrrr
        var leftConfig = new TalonFXConfiguration();
        //set inverted here
        leftConfig.CurrentLimits.SupplyCurrentLimit = 30;
        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // we don't need this anymore: leftConfig.MotorOutput.PeakReverseDutyCycle = 0;

        var leftSlot0Config = leftConfig.Slot0;
        leftSlot0Config.kS = 0.5; // Add 0.25 V output to overcome static friction
        leftSlot0Config.kV = 0.15; // A velocity target of 1 rps results in 0.12 V output
        leftSlot0Config.kA = 0.02; // An acceleration of 1 rps/s requires 0.01 V output
        leftSlot0Config.kP = 0.4; // An error of 1 rps results in 0.11 V output
        leftSlot0Config.kI = 0.0; // no output for integrated error
        leftSlot0Config.kD = 0.0; // no output for error derivative

        var leftMotionMagicConfig = leftConfig.MotionMagic;
        leftMotionMagicConfig.MotionMagicAcceleration = 125; // Target acceleration of 400 rps/s (0.25 seconds to max)
        leftMotionMagicConfig.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)

        m_leftFlywheel.getConfigurator().apply(leftConfig);

        var rightConfig = new TalonFXConfiguration();

        rightConfig.CurrentLimits.SupplyCurrentLimit = 30;

        m_rightFlywheel.getConfigurator().apply(rightConfig);

        m_rightFlywheel.setControl(new Follower(Constants.LEFT_FLYWHEEL, true));
    }

    public void updateInputs(ShooterIOInputs inputs) {
        inputs.shooterVelocityMPS = (m_leftFlywheel.getVelocity().getValueAsDouble())*Constants.FLYWHEEL_CIRCUMFERENCE;
        inputs.shooterCurrentAmps = m_leftFlywheel.getSupplyCurrent().getValueAsDouble();
        inputs.shooterVoltage = m_leftFlywheel.getMotorVoltage().getValueAsDouble();
        inputs.motorPosition = m_leftFlywheel.getPosition().getValueAsDouble();
    }

    public void setVoltage(double volts){
        m_leftFlywheel.setVoltage(volts);
    }

    public void setFlywheelSpeed(double velocity){
        // m_leftFlywheel.setVoltage(velocity);

        // VELOCITY IN MPS
        velocity = velocity/Constants.FLYWHEEL_CIRCUMFERENCE;
        m_leftFlywheel.setControl(m_request.withVelocity(velocity));
    }

    public void stopFlywheel(){
        m_leftFlywheel.set(0);
    }
}
