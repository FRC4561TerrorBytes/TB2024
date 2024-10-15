// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ArmIO {

    @AutoLog
    public static class ArmIOInputs {
        public double armAbsoluteAngleRotations = 0.0;
        public double armRelativeAngleRotations = 0.0;
        public double armVelocityRadPerSec = 0.0;
        public double armAppliedVolts = 0.0;
        public double armCurrentAmps = 0.0;

        public double armSetpoint = 0.0;
    }

    public default void seedEncoders() {}

    public default void updateInputs(ArmIOInputs inputs) {}
    
    public default void setArmVoltage(double volts) {}

    public default void setArmSetpoint(double setpoint) {}

    public default boolean armAtSetpoint(){return false;}

    public default void incrementArmAngle(double inc) {}

    public default void decrementArmAngle(double inc) {}

    public default void stopArm() {}

    public default double getArmEncoderRotation(){return 0.0;}

    public default void nudge(double degrees) {}

    public default double getAbsoluteRotations(){return 0.0;}

    public default boolean getDisconnect() {return false;}
}