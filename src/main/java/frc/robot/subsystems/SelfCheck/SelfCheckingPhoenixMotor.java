// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SelfCheck;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.subsystems.SelfCheck.SubSystemFaults;

public class SelfCheckingPhoenixMotor implements SelfChecking {
  /** Creates a new SelfCheckingPheonixMotor. */
  private String label;
  private TalonFX motor;
  private StatusSignal<Double> statusSignal;

  //Generates the Faults List Name and Motor
  public SelfCheckingPhoenixMotor(String label, TalonFX motor) {
    this.label = label;
    this.motor = motor;
    this.statusSignal = this.motor.getSupplyVoltage().clone();
  }

  @Override
  @AutoLogOutput(key = "(label) Faults")
  public List<SubSystemFaults> checkForFaults() {
    List<SubSystemFaults> faults = new ArrayList<>();

    if (motor.getFault_Hardware().getValue() == Boolean.TRUE) {
      faults.add(new SubSystemFaults(String.format("[%s]: Hardware failure detected", label)));
    }
    if (motor.getFault_BootDuringEnable().getValue() == Boolean.TRUE) {
      faults.add(new SubSystemFaults(String.format("[%s]: Device booted while enabled", label)));
    }
    if (motor.getFault_DeviceTemp().getValue() == Boolean.TRUE) {
      faults.add(
          new SubSystemFaults(
              String.format("[%s]: Device temperature exceeded limit", label), true));
    }
    if (motor.getFault_FusedSensorOutOfSync().getValue() == Boolean.TRUE) {
      faults.add(new SubSystemFaults(String.format("[%s]: Remote sensor is out of sync", label)));
    }
    if (motor.getFault_OverSupplyV().getValue() == Boolean.TRUE) {
      faults.add(new SubSystemFaults(String.format("[%s]: Supply voltage exceeded limit", label)));
    }
    if (motor.getFault_ProcTemp().getValue() == Boolean.TRUE) {
      faults.add(
          new SubSystemFaults(String.format("[%s]: Processor temperature exceeded limit", label)));
    }
    if (motor.getFault_Undervoltage().getValue() == Boolean.TRUE) {
      faults.add(
          new SubSystemFaults(String.format("[%s]: Device supply voltage near brownout", label)));
    }
    if (motor.getFault_UnlicensedFeatureInUse().getValue() == Boolean.TRUE) {
      faults.add(new SubSystemFaults(String.format("[%s]: Unlicensed feature in use", label)));
    }
    if (motor.getFault_UnstableSupplyV().getValue() == Boolean.TRUE) {
      faults.add(new SubSystemFaults(String.format("[%s]: Supply voltage is unstable", label)));
    }

    this.statusSignal.refresh();
    if (this.statusSignal.getStatus() != StatusCode.OK) {
      faults.add(new SubSystemFaults(String.format("[%s]: device is unreachable", label)));
    }

    return faults;
  }

  //Converts Faults List to Array
  public String[] faultsInArray(){
    String[] arr = checkForFaults().toArray(new String[0]);
    return arr;
  }
}
