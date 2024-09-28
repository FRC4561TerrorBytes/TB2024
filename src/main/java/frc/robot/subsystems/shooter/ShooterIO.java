/// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ShooterIO {

    @AutoLog
    public static class ShooterIOInputs {
        public double shooterVoltage = 0.0;
        public double shooterVelocityMPS = 0.0;
        public double shooterCurrentAmps = 0.0;
        public double motorPosition = 0.0;
        public double motorSetpoint = 0.0;
    }
    
    /** Run open loop at the specified voltage. */
    public default void setVoltage(double volts) {}

    public default void updateInputs(ShooterIOInputs inputs) {};

    public default void setFlywheelSpeed(double velocity) {};

    public default void stopFlywheel() {};
}
