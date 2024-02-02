// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanism;

import org.littletonrobotics.junction.AutoLog;

public interface MechanismIO {

    @AutoLog
    public static class MechanismIOInputs {
        public double elevatorPositionMeters = 0.0;
        public double elevatorVelocityRadPerSec = 0.0;
        public double elevatorAppliedVolts = 0.0;
        public double[] elevatorCurrentAmps = new double[] {};

        public double armAngleDegrees = 0.0;
        public double armVelocityRadPerSec = 0.0;
        public double armAppliedVolts = 0.0;
        public double[] armCurrentAmps = new double[] {};

        public double elevatorSetpoint = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(MechanismIOInputs inputs) {}

    /** Run the drive motor at the specified voltage. */
    public default void setElevatorVoltage(double volts) {}

    public default void setArmVoltage(double volts) {}

    public default void setElevatorSetpoint(double setpoint) {}
}
