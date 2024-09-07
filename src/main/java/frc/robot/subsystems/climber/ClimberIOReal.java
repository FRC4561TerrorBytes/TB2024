package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ClimberIOReal implements ClimberIO {

    private final CANSparkMax climberMotor = new CANSparkMax(99, MotorType.kBrushless);


    public ClimberIOReal() {
        climberMotor.restoreFactoryDefaults();

        climberMotor.setSmartCurrentLimit(50, 20);
        climberMotor.setIdleMode(IdleMode.kBrake);

        climberMotor.burnFlash();
    }

    public void updateInputs(ClimberIOInputs inputs) {
        inputs.climberAppliedVolts = climberMotor.getAppliedOutput();
        inputs.climberCurrentAmps = climberMotor.getOutputCurrent();
    }

    public void setClimberSpeed(double speed) {
        climberMotor.set(speed);
    }

    public void stopClimber() {
        setClimberSpeed(0);
    }
}