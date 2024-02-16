// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants;

/** Add your docs here. */
public class ElevatorIOReal implements ElevatorIO{
    private final TalonFX m_leftRaiseMotor = new TalonFX(Constants.LEFT_RAISE_MOTOR);
    private final TalonFX m_rightRaiseMotor = new TalonFX(Constants.RIGHT_RAISE_MOTOR);

    private double elevatorSetPoint;

    private PIDController raiseMotorController = new PIDController(0.11, 0, 0);

    private final AnalogInput m_analogInput;

    public ElevatorIOReal(){

        var leftConfig = new TalonFXConfiguration();
        
        //set inverted here
        //Should below line remain?
        //leftConfig.Feedback.SensorToMechanismRatio = Constants.FLYWHEEL_CIRCUMFERENCE/60;
        leftConfig.CurrentLimits.SupplyCurrentLimit = 40;
        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        m_leftRaiseMotor.getConfigurator().apply(leftConfig);

        var rightConfig = new TalonFXConfiguration();
        //set inverted here
        //Should below line remain?
        //rightConfig.Feedback.SensorToMechanismRatio = Constants.FLYWHEEL_CIRCUMFERENCE/60;
        rightConfig.CurrentLimits.SupplyCurrentLimit = 40;
        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        m_rightRaiseMotor.getConfigurator().apply(rightConfig);

        m_rightRaiseMotor.setControl(new Follower(Constants.LEFT_RAISE_MOTOR, true));

        this.m_analogInput = new AnalogInput(0);
        m_analogInput.setAverageBits(2);

        elevatorSetPoint = m_leftRaiseMotor.getPosition().getValueAsDouble();
    }

    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.elevatorPositionMeters = m_leftRaiseMotor.getPosition().getValueAsDouble();
        inputs.elevatorVelocityRadPerSec = m_analogInput.getValue();
        inputs.elevatorAppliedVolts = m_leftRaiseMotor.getMotorVoltage().getValueAsDouble();
        inputs.elevatorCurrentAmps = new double[] {m_leftRaiseMotor.getSupplyCurrent().getValueAsDouble()};
        inputs.elevatorSetpoint = elevatorSetPoint;
    }

    public void setElevatorVoltage(double volts) {

    }

    public void setElevatorSetpoint(double setpoint) {
        elevatorSetPoint = setpoint;
    }

    public void goToSetPoint(){
        m_leftRaiseMotor.set(raiseMotorController.calculate(m_leftRaiseMotor.getPosition().getValueAsDouble(), elevatorSetPoint));
    }
}
