// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import frc.robot.Constants;

/** Add your docs here. */
public class ElevatorIOReal implements ElevatorIO{
    private final CANSparkMax m_leftRaiseMotor = new CANSparkMax(Constants.LEFT_RAISE_MOTOR, MotorType.kBrushless);
    private final CANSparkMax m_rightRaiseMotor = new CANSparkMax(Constants.RIGHT_RAISE_MOTOR, MotorType.kBrushless);
    private final RelativeEncoder m_leftRaiseEncoder;

    private double elevatorSetPoint;

    private ElevatorFeedforward feedforward = new ElevatorFeedforward(Constants.ELEVATOR_STATIC_GAIN, Constants.ELEVATOR_GRAVITY_GAIN, Constants.ELEVATOR_VELOCITY_GAIN, Constants.ELEVATOR_ACCELERATION_GAIN);

    private PIDController raiseMotorController = new PIDController(0.01, 0, 0);

    private final AnalogPotentiometer m_analogPotentiometer;

    private boolean leftMotorNoLongerGood;
    private boolean rightMotorNoLongerGood;

    public ElevatorIOReal(){

        REVLibError leftCurrent = m_leftRaiseMotor.setSmartCurrentLimit(40);
        m_leftRaiseMotor.setIdleMode(IdleMode.kBrake);
        if (leftCurrent != REVLibError.kOk)
        {
            leftMotorNoLongerGood = true;
        } else
        {
            leftMotorNoLongerGood = false;
        }
        Logger.recordOutput("selfCheck/leftElevatorMotor", !leftMotorNoLongerGood);

        REVLibError rightCurrent = m_rightRaiseMotor.setSmartCurrentLimit(40);
        m_rightRaiseMotor.setIdleMode(IdleMode.kBrake);
        if (rightCurrent != REVLibError.kOk)
        {
            rightMotorNoLongerGood = true;
        } else
        {
            rightMotorNoLongerGood = false;
        }
        Logger.recordOutput("selfCheck/rightElevatorMotor", !rightMotorNoLongerGood);

        m_leftRaiseEncoder = m_leftRaiseMotor.getEncoder();
        m_leftRaiseEncoder.setPositionConversionFactor(16.15);

        m_rightRaiseMotor.follow(m_rightRaiseMotor, true);

        AnalogInput input = new AnalogInput(0);

        this.m_analogPotentiometer = new AnalogPotentiometer(input, 0, 0);

        elevatorSetPoint = m_analogPotentiometer.get();
    }

    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.elevatorPositionMeters = m_analogPotentiometer.get();
        inputs.elevatorVelocityRadPerSec = m_leftRaiseEncoder.getVelocity();
        inputs.elevatorAppliedVolts = m_leftRaiseMotor.getBusVoltage();
        inputs.elevatorCurrentAmps = new double[] {m_leftRaiseMotor.getOutputCurrent()};
        inputs.elevatorSetpoint = elevatorSetPoint;

        Logger.recordOutput("Elevator String Pot", m_analogPotentiometer.get());
    }

    public void setElevatorVoltage(double volts) {

    }

    public void setElevatorSetpoint(double setpoint) {
        elevatorSetPoint = setpoint;
    }

    public void goToSetPoint(){
        m_leftRaiseMotor.set(raiseMotorController.calculate(m_leftRaiseEncoder.getPosition(), elevatorSetPoint) 
            + feedforward.calculate(m_leftRaiseEncoder.getVelocity()));
        //m_leftRaiseMotor.set(raiseMotorController.calculate(m_analogPotentiometer.get(), elevatorSetPoint) 
           // + feedforward.calculate(m_leftRaiseEncoder.getVelocity()));
    }
}
