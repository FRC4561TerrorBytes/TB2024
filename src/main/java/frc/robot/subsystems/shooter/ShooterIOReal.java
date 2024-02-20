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
        leftConfig.CurrentLimits.SupplyCurrentLimit = 20;
        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        var leftSlot0Config = leftConfig.Slot0;
        leftSlot0Config.kS = 0.3; // Add 0.25 V output to overcome static friction
        leftSlot0Config.kV = 0.05; // A velocity target of 1 rps results in 0.12 V output
        leftSlot0Config.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        leftSlot0Config.kP = 0.22; // An error of 1 rps results in 0.11 V output
        leftSlot0Config.kI = 0; // no output for integrated error
        leftSlot0Config.kD = 0; // no output for error derivative

        var leftMotionMagicConfig = leftConfig.MotionMagic;
        leftMotionMagicConfig.MotionMagicAcceleration = 10; // Target acceleration of 400 rps/s (0.25 seconds to max)
        leftMotionMagicConfig.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)

        m_leftFlywheel.getConfigurator().apply(leftConfig);

        m_rightFlywheel.setControl(new Follower(1, true));
    }

    public void updateInputs(ShooterIOInputs inputs) {
        inputs.shooterVelocityMPS = (m_leftFlywheel.getVelocity().getValueAsDouble())*Constants.FLYWHEEL_CIRCUMFERENCE;
        inputs.shooterCurrentAmps = new double[] {m_leftFlywheel.getSupplyCurrent().getValueAsDouble()};
    }

    public void setFlywheelSpeed(double velocity){
        //m_leftFlywheel.set(velocity);

        velocity = velocity/Constants.FLYWHEEL_CIRCUMFERENCE;
        m_leftFlywheel.setControl(m_request.withVelocity(velocity));
    }

    public void stopFlywheel(){
        setFlywheelSpeed(0);
    }
}
