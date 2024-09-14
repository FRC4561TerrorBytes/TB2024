package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ClimberIOReal implements ClimberIO {

    private final CANSparkMax climberMotor = new CANSparkMax(28, MotorType.kBrushless);
    private final RelativeEncoder encoder;


    public ClimberIOReal() {
        climberMotor.restoreFactoryDefaults();
        
        encoder = climberMotor.getEncoder();
        climberMotor.setSmartCurrentLimit(60, 20);
        climberMotor.setIdleMode(IdleMode.kBrake
        );

        climberMotor.burnFlash();
    }

    public void updateInputs(ClimberIOInputs inputs) {
        inputs.climberAppliedVolts = climberMotor.getAppliedOutput();
        inputs.climberCurrentAmps = climberMotor.getOutputCurrent();
        inputs.climberPosition = encoder.getPosition();
    }

    public void setClimberSpeed(double speed) {
        climberMotor.set(speed);
    }

    public void stopClimber() {
        setClimberSpeed(0);
    }
}