// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

  private final AnalogInput m_analogInput;

  /** Creates a new Elevator. */
  public ElevatorSubsystem() {
    this.m_analogInput = new AnalogInput(0);
    m_analogInput.setAverageBits(2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_analogInput.getValue();
    Logger.recordOutput("Analog Potentionmeter", m_analogInput.getValue());
  }
}
