// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class ArmIOSim implements ArmIO {
    private static final double LOOP_PERIOD_SECS = 0.02;

    private double armAppliedVolts = 0.0;
    private double armSetpoint = 0.0;

    private DCMotorSim armMotorSim = new DCMotorSim(DCMotor.getFalcon500(1), 50, 0.025);

    public void updateInputs(ArmIOInputs inputs) {
        armMotorSim.update(LOOP_PERIOD_SECS);

        inputs.armAbsoluteAngleDegrees = Units.radiansToDegrees(armMotorSim.getAngularPositionRad());
        inputs.armVelocityRadPerSec = armMotorSim.getAngularVelocityRadPerSec();
        inputs.armAppliedVolts = armAppliedVolts;
        inputs.armCurrentAmps = Math.abs(armMotorSim.getCurrentDrawAmps());

        inputs.armSetpoint = armSetpoint;
    } 

    public void setArmVoltage(double volts) {
        armAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        armMotorSim.setInputVoltage(armAppliedVolts);
    }

    public void setArmSetpoint(double setpoint) {
        armSetpoint = setpoint;
    }

    public void incrementArmAngle(double inc) {
        armSetpoint += inc;
    }

    public void decrementArmAngle(double inc) {
        armSetpoint -= inc;
    }
}
