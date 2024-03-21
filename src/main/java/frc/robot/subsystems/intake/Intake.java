// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  
  public double m_velocitySetpoint;

  /** Creates a new Intake. */
  public Intake(IntakeIO io) {
    this.io = io;

        switch (Constants.currentMode) {
            case REAL:
            case REPLAY:
                
                break;
            case SIM:

                break;
            default:
                
                break;
        }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
        Logger.processInputs("Intake/IO", inputs);
    }

    public void setIntakeSpeed(double velocity){
      io.setIntakeSpeed(velocity);
    }

    public void stopIntake(){
      io.stopIntake();
    }

    public boolean getIntakeBreak() {
      return inputs.noteInIntake;
    }
  }
