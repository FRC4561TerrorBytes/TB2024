package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public double climberAppliedVolts = 0.0;
        public double climberCurrentAmps = 0.0;
        public double climberPosition = 0.0;
        public boolean climberSwitchOne = false;
        public boolean climberSwitchTwo = false;
    }

    public default void updateInputs(ClimberIOInputs inputs) {};

    public default void setClimberSpeed(double speed) {};

    public default void stopClimber() {};
}
