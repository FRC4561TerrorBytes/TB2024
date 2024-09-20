package frc.robot.util;

import com.ctre.phoenix6.StatusCode;
import com.revrobotics.CANSparkBase.FaultID;

import frc.robot.subsystems.Leds;

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
            case OK:
                // This case covers the TalonFX reporting no issues
                firmwareAlert.set(false);
                disconnectAlert.set(false);
                break;            
            default:
                //Random disconnects return various status codes depending on bus topology
                disconnectAlert.set(true);
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

        if (faults != 0) {
            disconnectAlert.set(true);
        } else {
            disconnectAlert.set(false);
            currentAlert.set(false);
        }
    }
    
}
