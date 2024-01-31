// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged inputs;
  private SimpleMotorFeedforward elevatorFeedForward;
  private PIDController elevatorFeedback;
  private Double positionSetpoint;

  public Elevator(ElevatorIO io) {
    this.io = io;

    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        elevatorFeedForward = new SimpleMotorFeedforward(0.1, 0.13);
        elevatorFeedback = new PIDController(0.05, 0.0, 0.0);
        break;
      case SIM:
        elevatorFeedForward = new SimpleMotorFeedforward(0.0, 0.13);
        elevatorFeedback = new PIDController(0.1, 0, 0);
        break;
      default:
        elevatorFeedForward = new SimpleMotorFeedforward(0, 0);
        elevatorFeedback = new PIDController(0, 0, 0);
        break;
    }
  }

  public void runWithVoltage(double volts) {
    io.setElevatorVoltage(volts);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator/IO", inputs);
  }
}