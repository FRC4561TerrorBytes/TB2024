// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

    @AutoLog
    public static class ElevatorIOInputs {
        public double elevatorPositionMeters = 0.705;
        public double elevatorVelocityRadPerSec = 0.0;
        public double elevatorAppliedVolts = 0.0;
        public double[] elevatorCurrentAmps = new double[] {};

        public double elevatorSetpoint = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ElevatorIOInputs inputs) {}

    /** Run the drive motor at the specified voltage. */
    public default void setElevatorVoltage(double volts) {}

    public default void setElevatorSetpoint(double setpoint) {}

    public default void goToSetPoint() {}
}
