package frc.robot.subsystems.climber;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimberIOSim implements ClimberIO {
    public static final double LOOP_PERIOD_SECS = 0.02;

    private double climberAppliedVolts = 0.0;

    private DCMotorSim climberMotorSim = new DCMotorSim(DCMotor.getNEO(1), 100, 0.025);

    public void updateInputs(ClimberIOInputs inputs) {
        climberMotorSim.update(LOOP_PERIOD_SECS);

        inputs.climberAppliedVolts = climberAppliedVolts;
    }

    public void setClimberSpeed(double speed) {
        climberAppliedVolts = speed / 12;
        climberMotorSim.setInputVoltage(climberAppliedVolts);
    }

    public void stopClimber() {
        setClimberSpeed(0);
    }
}
