// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class SnapTo45 extends Command {

  private PIDController pidController = new PIDController(0.04, 0, 0.00);

  private Drive drive;

  private int degreesClosestTo = 0;

  private double angle;

  double rotationRate;

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

    angle = drive.getRotation().getDegrees() + 180;

    double closest = 999.0;

    if(Math.abs(angle - 315) < closest){
      closest = Math.abs(angle - 315);
      degreesClosestTo = 315;
    }
    if(Math.abs(angle - 225) < closest){
      closest = Math.abs(angle - 225);
      degreesClosestTo = 225;
    }
    if(Math.abs(angle - 135) < closest){
      closest = Math.abs(angle - 135);
      degreesClosestTo = 135;
    }
    if(Math.abs(angle + 45) < closest){
      closest = Math.abs(angle + 45);
      degreesClosestTo = 45;
    }

    pidController.setSetpoint(degreesClosestTo);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double rawAngle = drive.getRotation().getDegrees();

    rotationRate = pidController.calculate(rawAngle + 180);    
    //rotationRate += 1.2 * Math.signum(rotationRate);
    rotationRate = MathUtil.applyDeadband(rotationRate, 0.1);

    rotationRate = Math.copySign(rotationRate * rotationRate, rotationRate);

    drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0.0, rotationRate, drive.getRotation()));

    Logger.recordOutput("Snap45/Raw Angle", rawAngle + 180);
    Logger.recordOutput("Snap45/Rotation Rate", rotationRate);
    Logger.recordOutput("Snap45/Angle Setpoint", degreesClosestTo);
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
