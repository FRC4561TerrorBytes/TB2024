// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.List;
import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.SelfCheck.SelfChecking;
import frc.robot.subsystems.SelfCheck.SelfCheckingPhoenixMotor;

/** Add your docs here. */
public class ModuleIOTBSwerve implements ModuleIO{

    private static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    private static final double TURN_GEAR_RATIO = 150.0 / 7.0;

    private final TalonFX driveTalon;
    private final CANSparkMax turnSparkMax;
    private final CANcoder cancoder;

    private final SelfCheckingPhoenixMotor driveTrainSelfCheck;
    private final String errorLabel;

    private final StatusSignal<Double> drivePosition;
    private final Queue<Double> drivePositionQueue;
    private final StatusSignal<Double> driveVelocity;
    private final StatusSignal<Double> driveAppliedVolts;
    private final StatusSignal<Double> driveCurrent;
    private final StatusSignal<Double> turnAbsolutePosition;

    private final RelativeEncoder turnRelativeEncoder;

    private final boolean isTurnMotorInverted = true;
    private final Rotation2d absoluteEncoderOffset;

    public ModuleIOTBSwerve(int index) {
        switch (index) {
        case 0:
            driveTalon = new TalonFX(Constants.FRONT_LEFT_DRIVE_MOTOR);
            driveTalon.setInverted(Constants.FRONT_LEFT_DRIVE_MOTOR_INVERTED);
            turnSparkMax = new CANSparkMax(Constants.FRONT_LEFT_STEER_MOTOR, MotorType.kBrushless);
            cancoder = new CANcoder(Constants.FRONT_LEFT_STEER_ENCODER);
            absoluteEncoderOffset = new Rotation2d(Constants.FRONT_LEFT_STEER_OFFSET); 
            errorLabel = "Module0";
            break;
        case 1:
            driveTalon = new TalonFX(Constants.FRONT_RIGHT_DRIVE_MOTOR);
            driveTalon.setInverted(Constants.FRONT_LEFT_DRIVE_MOTOR_INVERTED);
            turnSparkMax = new CANSparkMax(Constants.FRONT_RIGHT_STEER_MOTOR, MotorType.kBrushless);
            cancoder = new CANcoder(Constants.FRONT_RIGHT_STEER_ENCODER);
            absoluteEncoderOffset = new Rotation2d(Constants.FRONT_RIGHT_STEER_OFFSET);
            errorLabel = "Module1"; 
            break;
        case 2:
            driveTalon = new TalonFX(Constants.BACK_LEFT_DRIVE_MOTOR);
            driveTalon.setInverted(Constants.BACK_LEFT_DRIVE_MOTOR_INVERTED);
            turnSparkMax = new CANSparkMax(Constants.BACK_LEFT_STEER_MOTOR, MotorType.kBrushless);
            cancoder = new CANcoder(Constants.BACK_LEFT_STEER_ENCODER);
            absoluteEncoderOffset = new Rotation2d(Constants.BACK_LEFT_STEER_OFFSET);
            errorLabel = "Module2"; 
            break;
        case 3:
            driveTalon = new TalonFX(Constants.BACK_RIGHT_DRIVE_MOTOR);
            driveTalon.setInverted(Constants.BACK_RIGHT_DRIVE_MOTOR_INVERTED);
            turnSparkMax = new CANSparkMax(Constants.BACK_RIGHT_STEER_MOTOR, MotorType.kBrushless);
            cancoder = new CANcoder(Constants.BACK_RIGHT_STEER_ENCODER);
            absoluteEncoderOffset = new Rotation2d(Constants.BACK_RIGHT_STEER_OFFSET);
            errorLabel = "Module3"; 
            break;
        default:
            throw new RuntimeException("Invalid module index");
        }

        //initialize and Log Self Check
        driveTrainSelfCheck = new SelfCheckingPhoenixMotor(errorLabel, driveTalon);
        driveTrainSelfCheck.checkForFaults();
        driveTrainSelfCheck.faultsInArray();
        Logger.recordOutput(errorLabel, driveTrainSelfCheck.faultsInArray());

        var driveConfig = new TalonFXConfiguration();
        driveConfig.CurrentLimits.StatorCurrentLimit = Constants.DRIVE_CURRENT_LIMIT;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveTalon.getConfigurator().apply(driveConfig);
        setDriveBrakeMode(true);
        
        turnSparkMax.restoreFactoryDefaults();

        turnSparkMax.setCANTimeout(250);

        cancoder.getConfigurator().apply(new CANcoderConfiguration());
        turnRelativeEncoder = turnSparkMax.getEncoder();

        turnSparkMax.setInverted(isTurnMotorInverted);
        turnSparkMax.setSmartCurrentLimit(Constants.TURN_CURRENT_LIMIT);
        turnSparkMax.enableVoltageCompensation(12.0);

        drivePosition = driveTalon.getPosition();
        drivePositionQueue =
            PhoenixOdometryThread.getInstance().registerSignal(driveTalon, driveTalon.getPosition());
        driveVelocity = driveTalon.getVelocity();
        driveAppliedVolts = driveTalon.getMotorVoltage();
        driveCurrent = driveTalon.getStatorCurrent();
        turnAbsolutePosition = cancoder.getAbsolutePosition();

        turnRelativeEncoder.setPosition(0.0);
        turnRelativeEncoder.setMeasurementPeriod(10);
        turnRelativeEncoder.setAverageDepth(2);

        turnSparkMax.setCANTimeout(0);

        turnSparkMax.burnFlash();

        BaseStatusSignal.setUpdateFrequencyForAll(
            Module.ODOMETRY_FREQUENCY, drivePosition);
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            driveVelocity,
            driveAppliedVolts,
            driveCurrent,
            turnAbsolutePosition);
        driveTalon.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            drivePosition,
            driveVelocity,
            driveAppliedVolts,
            driveCurrent,
            turnAbsolutePosition);
    
        inputs.drivePositionRad =
            Units.rotationsToRadians(drivePosition.getValueAsDouble()) / DRIVE_GEAR_RATIO;
        inputs.driveVelocityRadPerSec =
            Units.rotationsToRadians(driveVelocity.getValueAsDouble()) / DRIVE_GEAR_RATIO;
        inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
        inputs.driveCurrentAmps = new double[] {driveCurrent.getValueAsDouble()};

        inputs.turnAbsolutePosition =
            new Rotation2d(turnAbsolutePosition.getValueAsDouble())
                .minus(absoluteEncoderOffset);
        inputs.turnPosition =
            Rotation2d.fromRotations(turnRelativeEncoder.getPosition() / TURN_GEAR_RATIO);
        inputs.turnVelocityRadPerSec =
            Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
                / TURN_GEAR_RATIO;
        inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
        inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};
    }

    public void setDriveVoltage(double volts) {
        driveTalon.setControl(new VoltageOut(volts));
    }

    public void setTurnVoltage(double volts) {
        turnSparkMax.setVoltage(volts);
    }

    public void setDriveBrakeMode(boolean enable) {
        var config = new MotorOutputConfigs();
        config.Inverted = InvertedValue.CounterClockwise_Positive;
        config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        driveTalon.getConfigurator().apply(config);
    }

    public void setTurnBrakeMode(boolean enable) {
        turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }
}