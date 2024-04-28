// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

/**
 * IO Layer implementation for TalonFX drive/ NEO turn on SDS MK4i L2 swerve
 * modules
 */
public class ModuleIOTBSwerve implements ModuleIO {

    private static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0); // L2 gearing
    private static final double TURN_GEAR_RATIO = 150.0 / 7.0;

    private final TalonFX driveTalon;
    private final CANSparkMax turnSparkMax;
    private final CANcoder cancoder;

    private final String moduleLabel;

    private final StatusSignal<Double> drivePosition;
    private final StatusSignal<Double> driveVelocity;
    private final StatusSignal<Double> driveAppliedVolts;
    private final StatusSignal<Double> driveCurrent;
    private final StatusSignal<Double> turnAbsolutePosition;

    private Alert driveMotorDisconnectAlert;
    private Alert driveMotorFirmwareAlert;
    private Alert turnMotorDisconnectAlert;
    private Alert turnMotorBrownoutAlert;
    private Alert turnMotorCurrentAlert;

    private final RelativeEncoder turnRelativeEncoder;

    private final boolean isTurnMotorInverted = true;
    private InvertedValue isDriveMotorInverted = InvertedValue.CounterClockwise_Positive;
    private final Rotation2d absoluteEncoderOffset;

    /**
     * Create swerve module object, using hardware specific constants
     * @param index
     */
    public ModuleIOTBSwerve(int index) {
        switch (index) {
            case 0:
                driveTalon = new TalonFX(Constants.FRONT_LEFT_DRIVE_MOTOR);
                isDriveMotorInverted = Constants.FRONT_LEFT_DRIVE_MOTOR_INVERTED;
                turnSparkMax = new CANSparkMax(Constants.FRONT_LEFT_STEER_MOTOR, MotorType.kBrushless);
                cancoder = new CANcoder(Constants.FRONT_LEFT_STEER_ENCODER);
                absoluteEncoderOffset = new Rotation2d(Constants.FRONT_LEFT_STEER_OFFSET);
                moduleLabel = "Module0";
                break;
            case 1:
                driveTalon = new TalonFX(Constants.FRONT_RIGHT_DRIVE_MOTOR);
                isDriveMotorInverted = Constants.FRONT_RIGHT_DRIVE_MOTOR_INVERTED;
                turnSparkMax = new CANSparkMax(Constants.FRONT_RIGHT_STEER_MOTOR, MotorType.kBrushless);
                cancoder = new CANcoder(Constants.FRONT_RIGHT_STEER_ENCODER);
                absoluteEncoderOffset = new Rotation2d(Constants.FRONT_RIGHT_STEER_OFFSET);
                moduleLabel = "Module1";
                break;
            case 2:
                driveTalon = new TalonFX(Constants.BACK_LEFT_DRIVE_MOTOR);
                isDriveMotorInverted = Constants.BACK_LEFT_DRIVE_MOTOR_INVERTED;
                turnSparkMax = new CANSparkMax(Constants.BACK_LEFT_STEER_MOTOR, MotorType.kBrushless);
                cancoder = new CANcoder(Constants.BACK_LEFT_STEER_ENCODER);
                absoluteEncoderOffset = new Rotation2d(Constants.BACK_LEFT_STEER_OFFSET);
                moduleLabel = "Module2";
                break;
            case 3:
                driveTalon = new TalonFX(Constants.BACK_RIGHT_DRIVE_MOTOR);
                isDriveMotorInverted = Constants.BACK_RIGHT_DRIVE_MOTOR_INVERTED;
                turnSparkMax = new CANSparkMax(Constants.BACK_RIGHT_STEER_MOTOR, MotorType.kBrushless);
                cancoder = new CANcoder(Constants.BACK_RIGHT_STEER_ENCODER);
                absoluteEncoderOffset = new Rotation2d(Constants.BACK_RIGHT_STEER_OFFSET);
                moduleLabel = "Module3";
                break;
            default:
                throw new RuntimeException("Invalid module index");
        }

        cancoder.getConfigurator().apply(new CANcoderConfiguration());

        // Set current limit configs for drive motor
        var driveConfig = new TalonFXConfiguration();
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveConfig.CurrentLimits.SupplyCurrentLimit = Constants.DRIVE_CURRENT_LIMIT;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.CurrentLimits.StatorCurrentLimit = Constants.DRIVE_STATOR_CURRENT_LIMIT;
        driveTalon.getConfigurator().apply(driveConfig);

        setDriveBrakeMode(true, isDriveMotorInverted);

        // Set config for turn motor
        turnSparkMax.restoreFactoryDefaults();
        turnSparkMax.setCANTimeout(250);
        turnRelativeEncoder = turnSparkMax.getEncoder();
        turnSparkMax.setInverted(isTurnMotorInverted);
        turnSparkMax.setSmartCurrentLimit(Constants.TURN_CURRENT_LIMIT);
        turnSparkMax.enableVoltageCompensation(12.0);
        turnSparkMax.setIdleMode(IdleMode.kBrake);

        REVLibError turnCurrent = turnSparkMax.setSmartCurrentLimit(Constants.TURN_CURRENT_LIMIT);
        boolean turnSparkMaxNoLongerGood;
        if (turnCurrent != REVLibError.kOk) {
            turnSparkMaxNoLongerGood = true;
        } else {
            turnSparkMaxNoLongerGood = false;
        }
        Logger.recordOutput("selfCheck/turnSparkMax{errorLabel}", !turnSparkMaxNoLongerGood);

        // Return initial status signals for Phoenix6 devices
        drivePosition = driveTalon.getPosition();
        driveVelocity = driveTalon.getVelocity();
        driveAppliedVolts = driveTalon.getMotorVoltage();
        driveCurrent = driveTalon.getSupplyCurrent();
        turnAbsolutePosition = cancoder.getAbsolutePosition();

        // Initialize relative encoder in turn motors
        turnRelativeEncoder.setPosition(0.0);
        turnRelativeEncoder.setMeasurementPeriod(10);
        turnRelativeEncoder.setAverageDepth(2);

        turnSparkMax.setCANTimeout(0);
        turnSparkMax.burnFlash();

        // Set update frequencies for TalonFX status signals, position at 2x frequency of other data
        BaseStatusSignal.setUpdateFrequencyForAll(
                100.0, drivePosition);
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

        // Report status code to AdvantageAlerts
        reportStatusCodeFault(drivePosition.getStatus(), moduleLabel);
        reportSparkMaxFault(moduleLabel, turnSparkMax);

        //Drive motor inputs
        inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble()) / DRIVE_GEAR_RATIO;
        inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble()) / DRIVE_GEAR_RATIO;
        inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
        inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

        //Turn motor / CANcoder positions
        inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
                .minus(absoluteEncoderOffset);
        inputs.turnPosition = Rotation2d.fromRotations(turnRelativeEncoder.getPosition() / TURN_GEAR_RATIO);
        inputs.turnVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
                / TURN_GEAR_RATIO;

        inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
        inputs.turnCurrentAmps = turnSparkMax.getOutputCurrent();

    }

    public void setDriveVoltage(double volts) {
        driveTalon.setControl(new VoltageOut(volts));
    }

    public void setTurnVoltage(double volts) {
        turnSparkMax.setVoltage(volts);
    }

    /**
     * Set inversion and brake mode for drive TalonFX
     * 
     * @param enable
     * @param inversion
     */
    public void setDriveBrakeMode(boolean enable, InvertedValue inversion) {
        var config = new MotorOutputConfigs();
        config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        config.Inverted = inversion;
        driveTalon.getConfigurator().apply(config);
    }

    public void setTurnBrakeMode(boolean enable) {
        turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }

    public TalonFX getDriveTalon() {
        return driveTalon;
    }

    /**
     * Report faults to AdvantageAlerts plugin, only report issues with CAN
     * connection, configs, firmware
     * 
     * @param statusCode
     * @param moduleLabel
     */
    public void reportStatusCodeFault(StatusCode statusCode, String moduleLabel) {
        // TODO: Better examine status codes thrown from common errors

        switch (statusCode) {
            case EcuIsNotPresent:
            case CouldNotRetrieveV6Firmware:
            case InvalidParamValue:
                // This case covers the TalonFX not existing, not sure it captures every
                // disconnect
                driveMotorDisconnectAlert = new Alert("Drive Motors", moduleLabel + ": is not present on CAN",
                        AlertType.ERROR);
                driveMotorDisconnectAlert.set(true);
                break;
            case NoConfigs:
                // This case should trigger when the TalonFX reboots improperly when code does
                // not reboot, such as brownouts
                driveMotorDisconnectAlert = new Alert("Drive Motors", moduleLabel + ": Does not have valid config",
                        AlertType.ERROR);
                driveMotorDisconnectAlert.set(true);
                break;
            case RxTimeout:
                // This should cover issues with CAN latency while connection is good, and catch
                // disconnects
                driveMotorDisconnectAlert = new Alert("Drive Motors",
                        moduleLabel + ": CAN frame not recieved/too stale", AlertType.ERROR);
                driveMotorDisconnectAlert.set(true);
            case ApiTooOld:
            case AppTooOld:
            case FirmwareTooNew:
            case FirmwareVersNotCompatible:
            case FirmVersionCouldNotBeRetrieved:
                // Old firmware and issues with firmware getting corrupted
                driveMotorFirmwareAlert = new Alert("Drive Motors", moduleLabel + ": has incorrect firmware",
                        AlertType.WARNING);
                driveMotorFirmwareAlert.set(true);
                break;
            case OK:
                // This case covers the TalonFX reporting no issues
                driveMotorFirmwareAlert.set(false);
                driveMotorDisconnectAlert.set(false);
                break;
            default:
                break;
        }

    }

    /**
     * Report faults with SparkMax API
     * 
     * @param moduleLabel
     * @param turnSparkMax
     */
    public void reportSparkMaxFault(String moduleLabel, CANSparkMax turnSparkMax) {
        // you should probably just call getFaults and do a bit mask to not repeatedly call the spark max API
        boolean CANfault = turnSparkMax.getFault(FaultID.kCANRX) || turnSparkMax.getFault(FaultID.kCANTX); // may be worthwhile to have separate Rx/Tx faults to help debug wiring
        boolean brownout = turnSparkMax.getFault(FaultID.kBrownout);
        boolean motorFault = turnSparkMax.getFault(FaultID.kMotorFault) || turnSparkMax.getFault(FaultID.kOvercurrent);

        if (CANfault) {
            turnMotorDisconnectAlert = new Alert("Turn Motors", moduleLabel + ": has CAN Rx/Tx fault", AlertType.ERROR);
            turnMotorDisconnectAlert.set(true);
        } else if (brownout) {
            turnMotorBrownoutAlert = new Alert("Turn Motors", moduleLabel + ": has brownout fault", AlertType.WARNING);
            turnMotorBrownoutAlert.set(true);
        } else if (motorFault) {
            turnMotorCurrentAlert = new Alert("Turn Motors", moduleLabel + ": has motor/overcurrent fault",
                    AlertType.WARNING);
            turnMotorCurrentAlert.set(true);
        } else {
            turnMotorDisconnectAlert.set(false);
            turnMotorBrownoutAlert.set(false);
            turnMotorCurrentAlert.set(false);
        }
    }
}