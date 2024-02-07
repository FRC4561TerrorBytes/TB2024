// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanism.Elevator;

public class ElevatorUpCommand extends Command {

  private Elevator elevator;
  /** Creates a new ElevatorUpCommand. */
  public ElevatorUpCommand(Elevator mech) {
    elevator = mech;
    addRequirements(mech);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.setElevatorSetpoint(0.419);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.elevatorFeedback.atSetpoint();
  }
}
