// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanism;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

/** Add your docs here. */
public class ElevatorIOSim implements ElevatorIO {
    private static final double LOOP_PERIOD_SECS = 0.02;

    private double elevatorAppliedVolts = 0.0;
    private double elevatorSetpoint = 0.0;

    private DCMotorSim elevatorMotorSim = new DCMotorSim(DCMotor.getFalcon500(1), Constants.ELEVATOR_MOTOR_GEAR_RATIO, 0.025);

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        elevatorMotorSim.update(LOOP_PERIOD_SECS);

        inputs.elevatorPositionMeters = getMetersPerDegree();
        inputs.elevatorVelocityRadPerSec = elevatorMotorSim.getAngularVelocityRadPerSec();
        inputs.elevatorAppliedVolts = elevatorAppliedVolts;
        inputs.elevatorCurrentAmps = new double[] {Math.abs(elevatorMotorSim.getCurrentDrawAmps())};

        inputs.elevatorSetpoint = elevatorSetpoint;
    }

    public double getMetersPerDegree() {
        double motorOutput = Units.radiansToDegrees(elevatorMotorSim.getAngularPositionRad());
        return Units.inchesToMeters(Constants.ELEVATOR_RATIO) * motorOutput;
    }

    public void setElevatorVoltage(double volts) {
        elevatorAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        elevatorMotorSim.setInputVoltage(elevatorAppliedVolts);
    }

    public void setElevatorSetpoint(double setpoint) {
        elevatorSetpoint = setpoint;
    }
}
