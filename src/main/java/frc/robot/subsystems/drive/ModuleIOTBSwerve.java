// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

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

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

/** Add your docs here. */
public class ModuleIOTBSwerve {

    private static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    private static final double TURN_GEAR_RATIO = 150.0 / 7.0;

    private final TalonFX driveTalon;
    private final CANSparkMax turnSparkMax;
    private final CANcoder cancoder;

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
            break;
        case 1:
            driveTalon = new TalonFX(Constants.FRONT_RIGHT_DRIVE_MOTOR);
            driveTalon.setInverted(Constants.FRONT_LEFT_DRIVE_MOTOR_INVERTED);
            turnSparkMax = new CANSparkMax(Constants.FRONT_RIGHT_STEER_MOTOR, MotorType.kBrushless);
            cancoder = new CANcoder(Constants.FRONT_RIGHT_STEER_ENCODER);
            absoluteEncoderOffset = new Rotation2d(Constants.FRONT_RIGHT_STEER_OFFSET); 
            break;
        case 2:
            driveTalon = new TalonFX(Constants.BACK_LEFT_DRIVE_MOTOR);
            driveTalon.setInverted(Constants.BACK_LEFT_DRIVE_MOTOR_INVERTED);
            turnSparkMax = new CANSparkMax(Constants.BACK_LEFT_STEER_MOTOR, MotorType.kBrushless);
            cancoder = new CANcoder(Constants.BACK_LEFT_STEER_ENCODER);
            absoluteEncoderOffset = new Rotation2d(Constants.BACK_LEFT_STEER_OFFSET); 
            break;
        case 3:
            driveTalon = new TalonFX(Constants.BACK_RIGHT_DRIVE_MOTOR);
            driveTalon.setInverted(Constants.BACK_RIGHT_DRIVE_MOTOR_INVERTED);
            turnSparkMax = new CANSparkMax(Constants.BACK_RIGHT_STEER_MOTOR, MotorType.kBrushless);
            cancoder = new CANcoder(Constants.BACK_RIGHT_STEER_ENCODER);
            absoluteEncoderOffset = new Rotation2d(Constants.BACK_RIGHT_STEER_OFFSET); 
            break;
        default:
            throw new RuntimeException("Invalid module index");
        }

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
