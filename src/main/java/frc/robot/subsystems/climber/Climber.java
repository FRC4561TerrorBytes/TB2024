// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  private ClimberIO io;
  private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  public Climber(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber/IO", inputs);
  }

  public void setClimberSpeed(double speed) {
    io.setClimberSpeed(speed);
  }

  public void stopClimber() {
    io.stopClimber();
  }

  public boolean getLimitSwitchStatus() {
    return inputs.climberLimitSwitch;
  }

  public double getClimberPosition() {
    return inputs.climberPosition;
  }

  public Command prepareClimber() {
    return new RunCommand(() -> setClimberSpeed(1), this).until(() -> getClimberPosition() > 120);
  }

  public Command climb() {
    return new RunCommand(() -> setClimberSpeed(1), this).until(() -> getLimitSwitchStatus());
  }
}
