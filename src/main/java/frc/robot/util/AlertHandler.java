package frc.robot.util;

import com.ctre.phoenix6.StatusCode;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkMax;

public class AlertHandler {

     /**
     * Report faults to AdvantageAlerts plugin, only report issues with CAN
     * connection, configs, firmware
     * 
     * @param statusCode
     * @param moduleLabel
     */
    public static void reportStatusCodeFault(StatusCode statusCode, String moduleLabel, Alert disconnectAlert, Alert firmwareAlert) {

        switch (statusCode) {
            case EcuIsNotPresent:
            case CouldNotRetrieveV6Firmware:
            case InvalidParamValue:
                // This case covers the TalonFX not existing, not sure it captures every
                // disconnect
                disconnectAlert.set(true);
                break;
            case NoConfigs:
                // This case should trigger when the TalonFX reboots improperly when code does
                // not reboot, such as brownouts

                disconnectAlert.set(true);
                break;
            case RxTimeout:
                // This should cover issues with CAN latency while connection is good, and catch
                // disconnects
                disconnectAlert.set(true);
                break;
            case CanMessageStale: 
                // Catches most CAN disconnects while the TalonFX is booted up
                disconnectAlert.set(true);
                break;
            case ApiTooOld:
            case AppTooOld:
            case FirmwareTooNew:
            case FirmwareVersNotCompatible:
            case FirmVersionCouldNotBeRetrieved:
                // Old firmware and issues with firmware getting corrupted
                firmwareAlert.set(true);
                break;
            case OK:
                // This case covers the TalonFX reporting no issues
                firmwareAlert.set(false);
                disconnectAlert.set(false);
                break;
            default:
                break;
        }

    }

    /**
     * Report faults with SparkMax API
     * 
     * @param moduleLabel
     * @param sparkMax
     */
    public static void reportSparkMaxFault(String moduleLabel, CANSparkMax sparkMax, Alert disconnectAlert, Alert currentAlert) {
        
        short faults = sparkMax.getFaults();
        boolean CANfault = sparkMax.getFault(FaultID.kCANRX) || sparkMax.getFault(FaultID.kCANTX) || sparkMax.getFault(FaultID.kBrownout); // may be                                                                                                  // wiring
        boolean motorFault = sparkMax.getFault(FaultID.kMotorFault) || sparkMax.getFault(FaultID.kOvercurrent);

        if (CANfault || faults != 0) {
            disconnectAlert.set(true);
        }  else if (motorFault) {
            currentAlert.set(true);
        } else {
            disconnectAlert.set(false);
            currentAlert.set(false);
        }
    }
    
}
