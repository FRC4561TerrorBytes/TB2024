// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class ElevatorIOSim implements ElevatorIO {
    private static final double LOOP_PERIOD_SECS = 0.02;

    private double elevatorAppliedVolts = 0.0;

    private DCMotorSim rightMotorSim = new DCMotorSim(DCMotor.getFalcon500(1), 16.15, 0.025);

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        rightMotorSim.update(LOOP_PERIOD_SECS);

        inputs.elevatorPositionMeters = Units.radiansToDegrees(rightMotorSim.getAngularPositionRad());
        inputs.elevatorVelocityRadPerSec = rightMotorSim.getAngularVelocityRadPerSec();

        inputs.elevatorAppliedVolts = elevatorAppliedVolts;
        inputs.elevatorCurrentAmps = new double[] {Math.abs(rightMotorSim.getCurrentDrawAmps())};
    }

    public void setElevatorVoltage(double volts) {
        elevatorAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        rightMotorSim.setInputVoltage(elevatorAppliedVolts);
    }
}
