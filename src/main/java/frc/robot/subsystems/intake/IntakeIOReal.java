// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;

import frc.robot.Constants;
import frc.robot.subsystems.Leds;
import frc.robot.util.Alert;
import frc.robot.util.AlertHandler;
import frc.robot.util.Alert.AlertType;

/** Add your docs here. */
public class IntakeIOReal implements IntakeIO{

    private final CANSparkMax m_frontIntake = new CANSparkMax(Constants.FRONT_INTAKE_MOTOR, MotorType.kBrushless);
    private final DigitalInput beamBreak = new DigitalInput(2);

    private Alert intakeMotorDisconnectAlert;
    private Alert intakeMotorCurrentAlert;
    private boolean alertActive;

      public IntakeIOReal() {

        // Restore front/back factory defaults
        m_frontIntake.restoreFactoryDefaults();

        // Idle phase for front and back
        m_frontIntake.setIdleMode(IdleMode.kCoast);

        // Limit of currents front/back
        REVLibError frontCurrent = m_frontIntake.setSmartCurrentLimit(35);
        m_frontIntake.setSmartCurrentLimit(35);
        boolean frontIntakeNoLongerGood;
        if ( frontCurrent!= REVLibError.kOk)
        {
            frontIntakeNoLongerGood = true;
        } else
        {
            frontIntakeNoLongerGood = false;
        }
        Logger.recordOutput("selfCheck/frontIntake", !frontIntakeNoLongerGood);
        
        //Voltage compensation
        m_frontIntake.enableVoltageCompensation(12.0);

        //Set inverted
        m_frontIntake.setInverted(true);

        m_frontIntake.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 50);
        m_frontIntake.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 50);

        m_frontIntake.burnFlash();   
        
        intakeMotorDisconnectAlert = new Alert("Intake Alert", "Intake motor is not present on CAN", AlertType.ERROR);
        intakeMotorCurrentAlert = new Alert("Intake Alert", "Intake motor has motor/overcurrent fault", AlertType.WARNING);

    }

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeAppliedVolts = m_frontIntake.getAppliedOutput();
        inputs.intakeCurrentAmps =  m_frontIntake.getOutputCurrent();
        inputs.noteInIntake = !beamBreak.get();

        Leds.getInstance().noteInIntake = !beamBreak.get();

        AlertHandler.reportSparkMaxFault("Intake Alert", m_frontIntake, intakeMotorDisconnectAlert, intakeMotorCurrentAlert);
    };

    @Override
    public boolean getDisconnect(){
        return intakeMotorDisconnectAlert.getState();
    }

    public void setIntakeSpeed(double velocity) {
        m_frontIntake.set(velocity);
    };

    public void stopIntake() {
        setIntakeSpeed(0);
    };
}
