// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.drive.Drive;

public class FaceSpeaker extends Command {

  private final PIDController controller;
  private static double kP = 0;
  private static double kI = 0;
  private static double kD = 0;
  private static double toleranceDegrees = 0;

  private Drive drive;

  public FaceSpeaker(Drive drive) {
    addRequirements(drive);
    this.drive = drive;

    switch (Constants.currentMode) {
      case REAL:
        kP = 0.08;
        kI = 0.0;
        kD = 0.0;
        toleranceDegrees = 1.0;
        break;
      default: // for SIM
        kP = 0.1;
        kI = 0.0;
        kD = 0.001;
        toleranceDegrees = 1.5;
        break;
    }

    controller = new PIDController(kP, kI, kD, 0.02);
    controller.setTolerance(toleranceDegrees);
    controller.enableContinuousInput(-180, 180);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.reset();
    controller.setSetpoint(drive.getRotationToSpeaker().getDegrees());

    Leds.getInstance().autoShootEndAngle = controller.getSetpoint() + 180;
    Leds.getInstance().autoShootStartAngle = drive.getPose().getRotation().getDegrees() + 180;
    Leds.getInstance().autoShootCommand = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //30 degrees per second max rotation speed
    double rotationSpeed = MathUtil.clamp(controller.calculate(drive.getPose().getRotation().getDegrees()), -30, 30);

    Leds.getInstance().autoShootCurrentAngle = drive.getPose().getRotation().getDegrees() + 180;

    drive.runVelocity(
      new ChassisSpeeds(0, 0,rotationSpeed));

    Logger.recordOutput("Speaker Rot/TurnActive", true);
    Logger.recordOutput("Speaker Rot/Current Rot", drive.getPose().getRotation().getDegrees());
    Logger.recordOutput("Speaker Rot/Turn X Goal", controller.getSetpoint());
    Logger.recordOutput("Speaker Rot/TurnSpeed", rotationSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted)
      drive.stopWithX();
    else
      drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }
}
