// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Add your docs here. */
public class Arm extends SubsystemBase {
    private ArmIO io;
    private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    private SimpleMotorFeedforward armFeedforward;
    public PIDController armFeedback; 

    public Arm(ArmIO io) {
        this.io = io;

        switch (Constants.currentMode) {
        case REAL:
        case REPLAY:
            armFeedforward = new SimpleMotorFeedforward(0.1, 0.13);
            armFeedback = new PIDController(0.5, 0, 0.0);
            break;
        case SIM:
            armFeedforward = new SimpleMotorFeedforward(0.0, 0.35);
            armFeedback = new PIDController(0.4, 0, 0.0075);
            break;
        default:
            armFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
            armFeedback = new PIDController(0.0, 0.0, 0.0);
            break;
        }
    }

    public void runArmWithVoltage(double volts) {
        io.setArmVoltage(volts);
    }

    public void setArmSetpoint(double setpoint) {
        io.setArmSetpoint(setpoint);
    }
    
    public double getArmAngleDegrees() {
        return inputs.armAbsoluteAngleDegrees;
    }

    public void incrementArmAngle(double inc) {
        io.incrementArmAngle(inc);
    }
    
    public void decrementArmAngle(double inc) {
        io.decrementArmAngle(inc);
    }

    public void seedEncoders() {
        io.seedEncoders();
    }

    public void stopArm() {
        io.setArmVoltage(0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm/IO", inputs);
    }
}
