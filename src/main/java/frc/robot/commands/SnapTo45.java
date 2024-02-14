// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class SnapTo45 extends Command {

  private PIDController pidController = new PIDController(0, 0, 0);

  private Drive drive;

  @AutoLogOutput(key = "Snap45/Angle Setpoint") 
  private int degreesClosestTo = 0;

  @AutoLogOutput(key = "Snap45/Rotate From")
  private double angle;

  public SnapTo45(Drive drive) {
    this.drive = drive;

    pidController.enableContinuousInput(-180, 180);
    pidController.setTolerance(1);

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();

    angle = drive.getRotation().getDegrees();

    double closest = 999.0;

    if(Math.abs(angle - 315) < closest){
      closest = Math.abs(angle - 270);
      degreesClosestTo = 270;
    }
    if(Math.abs(angle - 225) < closest){
      closest = Math.abs(angle - 180);
      degreesClosestTo = 180;
    }
    if(Math.abs(angle - 135) < closest){
      closest = Math.abs(angle - 90);
      degreesClosestTo = 90;
    }
    if(Math.abs(angle + 45) < closest){
      closest = Math.abs(angle + 0);
      degreesClosestTo = 0;
    }

    pidController.setSetpoint(degreesClosestTo);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rawAngle = drive.getRotation().getDegrees();

    DriveCommands.joystickDrive(drive, () -> 0.0, () -> 0.0,
      () -> pidController.calculate(rawAngle) + (1.2 * Math.signum(pidController.calculate(rawAngle))));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
