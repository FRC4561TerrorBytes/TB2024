package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import frc.robot.util.AlertHandler;

public class ArmIOReal implements ArmIO {
    private double armSetPoint = 0.0;
    private TalonFX m_armMotorLeft;
    private TalonFX m_armMotorRight;
    private DutyCycleEncoder encoder;

    // create a Motion Magic request, voltage output
    private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    private Alert armLeftMotorDisconnectAlert;
    private Alert armLeftMotorCurrentAlert; 

    private Alert armRightMotorDisconnectAlert;
    private Alert armRightMotorCurrentAlert; 

    public ArmIOReal() {
        

        m_armMotorLeft = new TalonFX(Constants.ARM_MOTOR_LEFT);
        m_armMotorRight = new TalonFX(Constants.ARM_MOTOR_RIGHT);

        var armConfig = new TalonFXConfiguration();

        armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        armConfig.CurrentLimits.SupplyCurrentLimit = Constants.ARM_CURRENT_LIMIT;
        armConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        // These soft limits are highly suspect
        armConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        armConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 11.75;

        armConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        armConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -12.5;

        // armConfig.Feedback.SensorToMechanismRatio = 50;

        // set slot 0 gains
        var slot0Configs = armConfig.Slot0;
        slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
        slot0Configs.kS = 0.28;// 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.3;// 2.82; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 5.75;// 0.0004; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0.7;// 0.0005; // no output for integrated error
        slot0Configs.kD = 0.15;// 0.0003; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic settings
        var motionMagicConfigs = armConfig.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 40; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 60; // Target acceleration of 160 rps/s (0.5 seconds)

        m_armMotorLeft.getConfigurator().apply(armConfig);
        m_armMotorRight.getConfigurator().apply(armConfig);

        m_armMotorLeft.setInverted(true);

        m_armMotorRight.setControl(new Follower(Constants.ARM_MOTOR_LEFT, true));
        // m_armMotorLeft.setPosition(getAbsoluteRotation());
        // m_armMotorRight.setPosition(getAbsoluteRotation());

         m_armMotorLeft.setPosition(-12.0);

        armLeftMotorDisconnectAlert = new Alert("Arm Alert", "Arm left motor is not present on CAN", AlertType.ERROR);
        armLeftMotorCurrentAlert = new Alert("Arm Alert", "Arm left motor has motor/overcurrent fault", AlertType.WARNING);

        armRightMotorDisconnectAlert = new Alert("Arm Alert", "Arm right motor is not present on CAN", AlertType.ERROR);
        armRightMotorCurrentAlert = new Alert("Arm Alert", "Arm right motor has motor/overcurrent fault", AlertType.WARNING);    
    }


    public void updateInputs(ArmIOInputs inputs) {
        inputs.armSetpoint = armSetPoint;
        
        inputs.armRelativeAngleRotations = m_armMotorLeft.getPosition().getValueAsDouble();
        inputs.armCurrentAmps = m_armMotorLeft.getSupplyCurrent().getValueAsDouble();
        Logger.recordOutput("FwdSoftLimit", m_armMotorLeft.getFault_ForwardSoftLimit().getValue().booleanValue());
        Logger.recordOutput("RevSoftLimit", m_armMotorLeft.getFault_ReverseSoftLimit().getValue().booleanValue());
        

        StatusCode leftCode = m_armMotorLeft.getAcceleration().getStatus();
        // AlertHandler.reportStatusCodeFault(leftCode, "Arm Alerts", armLeftMotorDisconnectAlert, armLeftMotorCurrentAlert);
        
        StatusCode rightCode = m_armMotorLeft.getAcceleration().getStatus();
        // AlertHandler.reportStatusCodeFault(rightCode, "Arm Alerts", armRightMotorDisconnectAlert, armRightMotorCurrentAlert);

    }

    public void seedEncoders() {
        m_armMotorLeft.setPosition(-12.0);
    }

    public void stopArm() {
        m_armMotorLeft.setVoltage(0);
    }

    public double getArmEncoderRotation() {
        return m_armMotorLeft.getPosition().getValueAsDouble();
    }

    public void setArmSetpoint(double angle) {
        // set target position
        armSetPoint = angle;
        m_armMotorLeft.setControl(m_request.withPosition(armSetPoint));
    }

    public boolean armAtSetpoint(){
        return Math.abs(getArmEncoderRotation() - armSetPoint) <= 0.2;
    }

    

    public void nudge(double degrees) {
        armSetPoint = m_armMotorLeft.getPosition().getValueAsDouble() + degrees;
        m_armMotorLeft.setControl(m_request.withPosition(armSetPoint));
    }

    @Override
    public boolean getDisconnect() {
        return armLeftMotorDisconnectAlert.getState() && armRightMotorDisconnectAlert.getState();
    }
}
