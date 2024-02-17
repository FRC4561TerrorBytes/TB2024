// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.NoteVisualizer;

/** Add your docs here. */
public class Shooter extends SubsystemBase {

    private ShooterIO io;
    private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public double m_velocitySetpoint;
    private double m_angle;
    private double m_height;
  
    private double m_pivotAngle;

    private double launchSpeedFeeder = 0.75;
    private double launchDelay = 1.0;

    public Shooter(ShooterIO io) {
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
        Logger.processInputs("Shooter/IO", inputs);

        Logger.recordOutput("Shooter/Angle", Units.radiansToDegrees(m_angle));
        Logger.recordOutput("Shooter/Height", m_height);
    }

  public double findVelocity(double x){
    return Math.sqrt((((
                    (-1*(9.8*Math.pow(-x, 2)))
              /((Constants.TARGET_Y + m_height)/(-x*Math.tan(m_angle))))
                /Math.pow(Math.cos(m_angle), 2)))
                                /2);
  }

  public double findTrajectoryPoint(double x, double distance){
    return((distance + x)*Math.tan(m_angle)-((9.8*Math.pow((distance + x), 2)) / (2*Math.pow(findVelocity(distance), 2) * Math.pow(Math.cos(m_angle), 2))));
  }

  public double findBestAngle(double distance){
    double add = 0.1;
    int searchLength = 100;
    if(findTrajectoryPoint(-Constants.TARGET_X, distance) < Constants.TARGET_X){
      add *= -1;
    }

    double startAngle = Units.radiansToDegrees(Math.atan((Constants.TARGET_Y-m_height)/(distance - Constants.TARGET_X)))+6;
    double[] angles = new double[searchLength];

    for(int i = 0; i < searchLength; i++){
      angles[i] = startAngle+(add*i);
    }

    //a high number aka mechanical advantage
    double closestError = 6328;
    int angleIndex = 0;

    double originalDistance = distance;

    for(int i = 0; i < searchLength; i++){
      m_angle = Units.degreesToRadians(angles[i]);
      m_height = Constants.ELEVATOR_PIVOT_HEIGHT-(Constants.ELEVATOR_PIVOT_LENGTH*Math.cos(m_angle - Constants.FLYWHEEL_OFFSET)) + (Constants.SHOOTER_FROM_ELEVATOR*Math.sin(m_angle));

      double xOffset = (Constants.ELEVATOR_PIVOT_LENGTH*Math.sin(m_angle - Constants.FLYWHEEL_OFFSET)) + (Constants.SHOOTER_FROM_ELEVATOR*Math.sin(Units.degreesToRadians(90) - m_angle));
      distance = originalDistance - xOffset + Constants.ELEVATOR_X_OFFSET;
      double error = Math.abs(findTrajectoryPoint(-Constants.TARGET_X, distance) - (Constants.TARGET_Y - m_height));

      if (error < closestError){
        closestError = error;
        angleIndex = i;
      }
    }
    return angles[angleIndex];
  }

  public double calculateShooter(double distance){
    m_angle = Units.degreesToRadians(findBestAngle(distance));

    double originalDistance = distance;
    double xOffset = (Constants.ELEVATOR_PIVOT_LENGTH*Math.sin(m_angle - Constants.FLYWHEEL_OFFSET)) + (Constants.SHOOTER_FROM_ELEVATOR*Math.sin(Units.degreesToRadians(90) - m_angle));
    distance = originalDistance - xOffset + Constants.ELEVATOR_X_OFFSET;
    m_velocitySetpoint = findVelocity(distance);

    m_height = Constants.ELEVATOR_PIVOT_HEIGHT-(Constants.ELEVATOR_PIVOT_LENGTH*Math.cos(m_angle - Constants.FLYWHEEL_OFFSET)) + (Constants.SHOOTER_FROM_ELEVATOR*Math.sin(m_angle));
    m_pivotAngle = m_angle - Constants.FLYWHEEL_OFFSET;

    return m_velocitySetpoint;
  }

  @AutoLogOutput(key = "Shooter/VelocitySetpoint")
  public double getVelocity(){
    return m_velocitySetpoint;
  }

  @AutoLogOutput(key = "Shooter/Pivot Angle")
  public double getPivotAngle(){
    return Units.radiansToDegrees(m_pivotAngle);
  }

  public void setFlywheelSpeed(double velocity){
    io.setFlywheelSpeed(velocity);
  }
  public void stopFlywheel(){
    io.stopFlywheel();
  }

  public boolean flywheelUpToSpeed(double mps){
    return inputs.shooterVelocityMPS >= mps - 0.2;
  }

  public boolean noteShot(){
    //return the beam break after the flywheels here
    return false;
  }

  /** Returns a command that launches a note. */
  public Command launchCommand() {
    return Commands.sequence(
            runOnce(
                () -> {
                  io.setFlywheelSpeed(m_velocitySetpoint);
                }),
            Commands.waitSeconds(launchDelay),
                NoteVisualizer.shoot(m_velocitySetpoint, Units.radiansToDegrees(m_angle)),
            Commands.idle())
        .finallyDo(
            () -> {
              io.setFlywheelSpeed(0.0);
            });
  }
}
