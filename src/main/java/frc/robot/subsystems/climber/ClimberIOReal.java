package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.subsystems.Leds;

public class ClimberIOReal implements ClimberIO {

    private final CANSparkMax climberMotor = new CANSparkMax(28, MotorType.kBrushless);
    private final RelativeEncoder encoder;
    private final SparkLimitSwitch limitSwitch;


    public ClimberIOReal() {
        climberMotor.restoreFactoryDefaults();
        
        encoder = climberMotor.getEncoder();
        climberMotor.setSmartCurrentLimit(15);
        climberMotor.setIdleMode(IdleMode.kBrake);

        limitSwitch = climberMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        limitSwitch.enableLimitSwitch(true);

        climberMotor.burnFlash();
    }

    public void updateInputs(ClimberIOInputs inputs) {
        inputs.climberAppliedVolts = climberMotor.getAppliedOutput();
        inputs.climberCurrentAmps = climberMotor.getOutputCurrent();
        inputs.climberPosition = encoder.getPosition();
        inputs.climberLimitSwitch = limitSwitch.isPressed();
        inputs.climberTempC = climberMotor.getMotorTemperature();

        if (limitSwitch.isPressed()) {
            Leds.getInstance().climbLimit = true;
        } else {
            Leds.getInstance().climbLimit = false;
        }
    }

    public void setClimberSpeed(double speed) {
        climberMotor.set(speed);
    }

    public void stopClimber() {
        setClimberSpeed(0);
    }
}