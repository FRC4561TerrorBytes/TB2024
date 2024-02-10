// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;

/** Add your docs here. */
public class IntakeIOReal implements IntakeIO{

    private final CANSparkMax m_frontIntake = new CANSparkMax(Constants.FRONT_INTAKE, MotorType.kBrushless);
    private final CANSparkMax m_backIntake = new CANSparkMax(Constants.BACK_INTAKE, MotorType.kBrushless);
    private final CANSparkMax m_intakeRotater = new CANSparkMax(Constants.INTAKE_ROTATOR, MotorType.kBrushless);
    private final CANSparkMax m_rotatorRoller = new CANSparkMax(Constants.ROTATOR_ROLLER, MotorType.kBrushless);
    private RelativeEncoder m_intakeEncoder;
    private SparkPIDController m_intakeController;
    private SparkAbsoluteEncoder m_intakeThroughboreEncoder;

      public IntakeIOReal() {

        // Restore front/back factory defaults
        m_frontIntake.restoreFactoryDefaults();
        m_backIntake.restoreFactoryDefaults();
        m_intakeRotater.restoreFactoryDefaults();
        m_rotatorRoller.restoreFactoryDefaults();

        // Idle phase for front and back
        m_frontIntake.setIdleMode(IdleMode.kBrake);
        m_intakeRotater.setIdleMode(IdleMode.kBrake);
        m_rotatorRoller.setIdleMode(IdleMode.kBrake);

        // Limit of currents front/back
        m_frontIntake.setSmartCurrentLimit(20);
        m_intakeRotater.setSmartCurrentLimit(20);
        m_rotatorRoller.setSmartCurrentLimit(20);

        //Voltage compensation
        m_frontIntake.enableVoltageCompensation(12.0);
        m_intakeRotater.enableVoltageCompensation(12.0);
        m_rotatorRoller.enableVoltageCompensation(12.0);

        //Set inverted
        m_frontIntake.setInverted(false);
        m_intakeRotater.setInverted(false);
        m_rotatorRoller.setInverted(false);

        m_backIntake.follow(m_frontIntake, false);

        //Get encoder
        m_intakeEncoder = m_intakeRotater.getEncoder();

        //Controller stuffs
        m_intakeController = m_intakeRotater.getPIDController();
        m_intakeController.setP(0);
        m_intakeController.setD(0);
        m_intakeController.setI(0);
        m_intakeController.setIZone(0);
        m_intakeController.setIZone(0.0);
        m_intakeController.setFF(0.0);
        m_intakeController.setOutputRange(0.0,1.0);

        //More encoders
        m_intakeThroughboreEncoder = m_intakeRotater.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        m_intakeThroughboreEncoder.setPositionConversionFactor(360.0);
    }

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeAppliedVolts = m_frontIntake.getAppliedOutput();
        inputs.barAngle = m_intakeThroughboreEncoder.getPosition();
        inputs.intakeCurrentAmps = new double[] {m_frontIntake.getOutputCurrent()};
    };

    public void setIntakeSpeed(double velocity) {
        m_frontIntake.set(velocity);
        m_rotatorRoller.set(velocity);
    };

    public void setBarAngle(double barAngle) {
        m_intakeController.setIAccum(0.0);
        m_intakeController.setReference(barAngle, CANSparkMax.ControlType.kPosition);
    }

    public void stopIntake() {
        setIntakeSpeed(0);
    };
}
