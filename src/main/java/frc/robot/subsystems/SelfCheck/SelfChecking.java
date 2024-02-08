// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SelfCheck;

import java.util.List;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.subsystems.SelfCheck.SubSystemFaults;

public interface SelfChecking {
  List<SubSystemFaults> checkForFaults();
} 
