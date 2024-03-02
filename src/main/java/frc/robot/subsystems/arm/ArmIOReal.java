package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;

public class ArmIOReal implements ArmIO {
    private double armSetPoint = 0.0;
    private TalonFX m_armMotorLeft;
    private TalonFX m_armMotorRight;
    private DutyCycleEncoder encoder;

public ArmIOReal () {
    encoder = new DutyCycleEncoder(0);
    // encoder.setDistancePerRo tation(360.0);
    encoder.setPositionOffset(0.666666);

    m_armMotorLeft = new TalonFX(Constants.ARM_MOTOR_LEFT);
    m_armMotorRight = new TalonFX(Constants.ARM_MOTOR_RIGHT);

    var armConfig = new TalonFXConfiguration();

    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    armConfig.CurrentLimits.SupplyCurrentLimit = Constants.ARM_CURRENT_LIMIT;
    armConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    armConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 170;
    armConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    armConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = -5;
    armConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    armConfig.Feedback.SensorToMechanismRatio = 0.122;

    // set slot 0 gains
    var slot0Configs = armConfig.Slot0;
    slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 5.64; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.08; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 0.001; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output

    // set Motion Magic settings
    var motionMagicConfigs = armConfig.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = 10; // Target acceleration of 160 rps/s (0.5 seconds)

    m_armMotorLeft.getConfigurator().apply(armConfig);

    m_armMotorLeft.setInverted(true);
    m_armMotorRight.setInverted(true);

    m_armMotorRight.setControl(new Follower(Constants.ARM_MOTOR_LEFT,true)); 
}

public void updateInputs(ArmIOInputs inputs) {
    inputs.armSetpoint = armSetPoint;
    inputs.armAbsoluteAngleDegrees = Units.rotationsToDegrees(encoder.getDistance());
    inputs.armRelativeAngleDegrees = m_armMotorLeft.getPosition().getValueAsDouble();
    inputs.armCurrentAmps = new double[] {m_armMotorLeft.getSupplyCurrent().getValueAsDouble()};
}

public void seedEncoders() {
    m_armMotorLeft.setPosition(encoder.getDistance());
    m_armMotorRight.setPosition(encoder.getDistance());
}

public void stopArm() {
    m_armMotorLeft.setVoltage(0);
}

public void setArmSetpoint(double angle){
    // create a Motion Magic request, voltage output
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    // set target position to 100 rotations
    m_armMotorLeft.setControl(m_request.withPosition(angle));
  }
}
