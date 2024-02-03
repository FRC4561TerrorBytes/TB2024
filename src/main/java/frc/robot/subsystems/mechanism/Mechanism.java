// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanism;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Mechanism extends SubsystemBase {

  private MechanismIO io;
  private MechanismIOInputsAutoLogged inputs = new MechanismIOInputsAutoLogged();
  private MechanismLigament2d m_elevator;
  private MechanismLigament2d m_arm;
  private MechanismLigament2d m_shooter;

  private SimpleMotorFeedforward elevatorFeedForward;
  private PIDController elevatorFeedback;

  private SimpleMotorFeedforward armFeedforward;
  private PIDController armFeedback;

  private double elevatorMinLenth = 0.5;

  public Mechanism(MechanismIO io) {
    this.io = io;

    Mechanism2d mech = new Mechanism2d(4, 4);

    MechanismRoot2d root = mech.getRoot("base", 2, 0);

    m_elevator = root.append(new MechanismLigament2d("elevator", elevatorMinLenth, 90));
    m_arm = m_elevator.append(new MechanismLigament2d("arm", 0.3, 90, 6, new Color8Bit(Color.kPurple)));
    m_shooter = m_arm.append(new MechanismLigament2d("shooter", 0.2, 120, 6, new Color8Bit(Color.kHotPink)));

    SmartDashboard.putData("Mech 2d", mech);

    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        elevatorFeedForward = new SimpleMotorFeedforward(0.1, 0.13);
        elevatorFeedback = new PIDController(0.05, 0.0, 0.0);
        armFeedforward = new SimpleMotorFeedforward(0.1, 0.13);
        armFeedback = new PIDController(0.5, 0, 0.0);
        break;
      case SIM:
        elevatorFeedForward = new SimpleMotorFeedforward(0.0, 0.25);
        elevatorFeedback = new PIDController(3.0, 0, 0.5);
        armFeedforward = new SimpleMotorFeedforward(0.0, 0.35);
        armFeedback = new PIDController(0.4, 0, 0.0075);
        break;
      default:
        elevatorFeedForward = new SimpleMotorFeedforward(0, 0);
        elevatorFeedback = new PIDController(0, 0, 0);
        armFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
        armFeedback = new PIDController(0.0, 0.0, 0.0);
        break;
    }
  }

  public void runElevatorWithVoltage(double volts) {
    io.setElevatorVoltage(volts);
  }

  public void runArmWithVoltage(double volts) {
    io.setArmVoltage(volts);
  }

  public void setElevatorSetpoint(double setpoint) {
    io.setElevatorSetpoint(setpoint);
  }

  public double getElevatorPositionMeters() {
    return inputs.elevatorPositionMeters;
  }

  public void setArmSetpoint(double setpoint) {
    io.setArmSetpoint(setpoint);
  }

  public double getArmAngleDegrees() {
    return inputs.armAngleDegrees;
  }

  public void incrementArmAngle(double inc) {
    io.incrementArmAngle(inc);
  }

  public void decrementArmAngle(double inc) {
    io.decrementArmAngle(inc);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator/IO", inputs);

    runElevatorWithVoltage(
      elevatorFeedForward.calculate(inputs.elevatorVelocityRadPerSec)
        + elevatorFeedback.calculate(inputs.elevatorPositionMeters, inputs.elevatorSetpoint));

    runArmWithVoltage(
      armFeedforward.calculate(inputs.armVelocityRadPerSec)
        + armFeedback.calculate(inputs.armAngleDegrees, inputs.armSetpoint));

    m_elevator.setLength(elevatorMinLenth + inputs.elevatorPositionMeters);
    m_arm.setAngle(inputs.armAngleDegrees);
  }
}