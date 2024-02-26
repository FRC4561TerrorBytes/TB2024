// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

/** Add your docs here. */
public class ShooterIOSim implements ShooterIO{
    private static final double LOOP_PERIOD_SECS = 0.02;

    private DCMotorSim shooterMotorSim = new DCMotorSim(DCMotor.getFalcon500(1), Constants.SHOOTER_MOTOR_GEAR_RATIO, 8.0);

    private SimpleMotorFeedforward shooterFeedforward = new SimpleMotorFeedforward(1.0, 0.1);
    private PIDController shooterFeedback = new PIDController(2000.0, 0.0, 300.0);

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        shooterMotorSim.update(LOOP_PERIOD_SECS);

        inputs.shooterVelocityMPS = shooterMotorSim.getAngularVelocityRPM() * Constants.FLYWHEEL_CIRCUMFERENCE / 60;
        inputs.shooterCurrentAmps = shooterMotorSim.getCurrentDrawAmps();

        shooterMotorSim.setInputVoltage(
            shooterFeedforward.calculate(
                shooterMotorSim.getAngularVelocityRPM() * Constants.FLYWHEEL_CIRCUMFERENCE / 60)
                + shooterFeedback.calculate(shooterMotorSim.getAngularVelocityRPM() * Constants.FLYWHEEL_CIRCUMFERENCE / 60));
    }

    public void setFlywheelSpeed(double velocity){
        shooterFeedback.setSetpoint(velocity);
    }
    public void stopFlywheel(){
        shooterMotorSim.setInputVoltage(0);
    }

}
