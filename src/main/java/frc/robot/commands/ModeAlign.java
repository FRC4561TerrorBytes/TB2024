// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GameMode;
import frc.robot.GameMode.Mode;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;

public class ModeAlign extends Command {

  private Drive drive;
  private Indexer indexer;
  private Intake intake;
  private Arm arm;
  private LEDSubsystem led;

  /** Creates a new ModeAlign. */
  public ModeAlign(Drive drive, Indexer indexer, Intake intake, Arm arm) {
    this.drive = drive;
    this.indexer = indexer;
    this.intake = intake;
    this.arm = arm;
    led = new LEDSubsystem();

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if (GameMode.getInstance().getCurrentMode().equals(Mode.SPEAKER)) {
        new FaceSpeaker(drive);
      } else if (GameMode.getInstance().getCurrentMode().equals(Mode.AMP)) {

      } else if (GameMode.getInstance().getCurrentMode().equals(Mode.TRAP)) {

      } else if (GameMode.getInstance().getCurrentMode().equals(Mode.INTAKING)) {
        new NoteAlign(drive, indexer, intake, arm);
      } else {

      }
      led.setColor(176,11,105);
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
