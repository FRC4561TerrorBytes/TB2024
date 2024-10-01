package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ClimberIOReal implements ClimberIO {

    private final CANSparkMax climberMotor = new CANSparkMax(28, MotorType.kBrushless);
    private final RelativeEncoder encoder;
    private final SparkLimitSwitch switchOne;
    private final SparkLimitSwitch switchTwo;


    public ClimberIOReal() {
        climberMotor.restoreFactoryDefaults();
        
        encoder = climberMotor.getEncoder();
        climberMotor.setSmartCurrentLimit(60, 20);
        climberMotor.setIdleMode(IdleMode.kBrake
        );

        switchOne = climberMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
        switchTwo = climberMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);

        switchOne.enableLimitSwitch(false);
        switchTwo.enableLimitSwitch(true);

        climberMotor.burnFlash();
    }

    public void updateInputs(ClimberIOInputs inputs) {
        inputs.climberAppliedVolts = climberMotor.getAppliedOutput();
        inputs.climberCurrentAmps = climberMotor.getOutputCurrent();
        inputs.climberPosition = encoder.getPosition();
        inputs.climberSwitchOne = switchOne.isPressed();
        inputs.climberSwitchTwo = switchTwo.isPressed();
    }

    public void setClimberSpeed(double speed) {
        climberMotor.set(speed);
    }

    public void stopClimber() {
        setClimberSpeed(0);
    }
}