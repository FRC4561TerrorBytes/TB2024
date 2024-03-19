// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.GameMode;
import frc.robot.GameMode.Mode;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommand extends Command {

  private Intake intake;
  private Indexer indexer;
  private Arm arm;

  public IntakeCommand(Intake intake, Indexer indexer, Arm arm) {
    this.intake = intake;
    this.indexer = indexer;
    this.arm = arm;

    addRequirements(intake, indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    GameMode.getInstance().setCurrentMode(Mode.INTAKING);
    arm.setArmSetpoint(Constants.ARM_STOW);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // intake.setBarAngle(Constants.INTAKE_LOW_POSITION);

    intake.setIntakeSpeed(Constants.INTAKE_SPEED);
    indexer.setIndexerSpeed(Constants.INDEXER_FEED_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // GameMode.getInstance().setCurrentMode(Mode.IDLE);
        new InstantCommand(() -> intake.stopIntake())
        .alongWith(new InstantCommand(() -> indexer.stopIndexer()));
              // intake.setBarAngle(Constants.INTAKE_HIGH_POSITION);
      //ADD LED STUFF
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return indexer.noteInIndexer();
  }
}
