// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax m_topRoller = new CANSparkMax(15, MotorType.kBrushless);
  private final CANSparkMax m_bottomRoller = new CANSparkMax(16, MotorType.kBrushless);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    m_topRoller.setInverted(false);
    m_bottomRoller.setInverted(false);
  }

  public void setRollerSpeed(double speed) {
    m_topRoller.set(speed);
    m_bottomRoller.set(speed);
  }

  public void stop() {
    setRollerSpeed(0.0);
  }

  @Override
  public void periodic() {

  }
}