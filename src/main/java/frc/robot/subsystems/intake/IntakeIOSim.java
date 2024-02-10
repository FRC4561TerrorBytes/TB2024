// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;

/** Add your docs here. */
public class IntakeIOSim implements IntakeIO{
    private static final double LOOP_PERIOD_SECS = 0.02;

    private double barSetPoint = 0.0;

    private double intakeAppliedVolts = 0.0;

    private DCMotorSim intakeMotorSim = new DCMotorSim(DCMotor.getFalcon500(1), Constants.INTAKE_MOTOR_GEAR_RATIO, 8.0);
    private DCMotorSim barMotorSim = new DCMotorSim(DCMotor.getFalcon500(1), Constants.BAR_MOTOR_GEAR_RATIO, 8.0);

    private SimpleMotorFeedforward intakeFeedforward = new SimpleMotorFeedforward(1.0, 0.1);
    private PIDController intakeFeedback = new PIDController(250.0, 0.0, 0.0);

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        intakeMotorSim.update(LOOP_PERIOD_SECS);

        inputs.intakeAppliedVolts = intakeAppliedVolts;
        inputs.barAngle = Units.radiansToDegrees(barMotorSim.getAngularPositionRad());
        inputs.intakeCurrentAmps = new double[] {Math.abs(intakeMotorSim.getCurrentDrawAmps())};
        inputs.barSetPoint = barSetPoint;
    }

    public void setIntakeSpeed(double volts){
        intakeAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        intakeMotorSim.setInputVoltage(intakeAppliedVolts);
    }

    public void setBarAngle(double setPoint){
        barSetPoint = setPoint;
    }
}
