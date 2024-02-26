// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {

  private IndexerIO io;
  private IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  /** Creates a new Indexer. */
  public Indexer(IndexerIO io) {
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
    io.updateInputs(inputs);
    Logger.processInputs("Indexer/IO", inputs);
  }

    public void setIndexerSpeed(double speed){
    io.setIndexerSpeed(speed);
  }
  public void stopIndexer(){
    io.stopIndexer();
  }

  public boolean noteInIndexer(){
    //return the beam breaks in the indexer here
    return io.getIndexerState();
  }
}