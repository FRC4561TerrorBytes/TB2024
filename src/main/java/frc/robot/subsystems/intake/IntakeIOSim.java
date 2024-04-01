// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

/** Add your docs here. */
public class IntakeIOSim implements IntakeIO{
    private static final double LOOP_PERIOD_SECS = 0.02;

    private double intakeAppliedVolts = 0.0;

    private DCMotorSim intakeMotorSim = new DCMotorSim(DCMotor.getFalcon500(1), Constants.INTAKE_MOTOR_GEAR_RATIO, 0.025);

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        intakeMotorSim.update(LOOP_PERIOD_SECS);

        inputs.intakeAppliedVolts = intakeAppliedVolts;
        inputs.intakeCurrentAmps = Math.abs(intakeMotorSim.getCurrentDrawAmps());
    }

    public void setIntakeSpeed(double volts){
        intakeAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        intakeMotorSim.setInputVoltage(intakeAppliedVolts);
    }
}
