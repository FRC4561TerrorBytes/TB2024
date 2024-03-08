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

public class SnapTo90 extends Command {

  private PIDController pidController = new PIDController(0.04, 0, 0.00);

  private Drive drive;

  private int degreesClosestTo = 0;

  private double angle;

  double rotationRate;

  public SnapTo90(Drive drive) {
    this.drive = drive;

    pidController.enableContinuousInput(0, 360);
    pidController.setTolerance(1);

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();

    angle = drive.getRotation().getDegrees() + 180;

    if(angle >= 225 && angle <= 315){
      degreesClosestTo = 270;
    }
    else if ((angle >= 315 && angle <= 360) || (angle <= 45 && angle >= 0))
    {
      degreesClosestTo = 0;
    }
    else if(angle > 45 && angle <= 135){
      degreesClosestTo = 90;
    }
    else if(angle > 135 && angle <= 225){
      degreesClosestTo = 180;
    }
    else{
      degreesClosestTo = 270;
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

    Logger.recordOutput("Snap90/Raw Angle", rawAngle + 180);
    Logger.recordOutput("Snap90/Rotation Rate", rotationRate);
    Logger.recordOutput("Snap90/Angle Setpoint", degreesClosestTo);
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
