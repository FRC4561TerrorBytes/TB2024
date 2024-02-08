// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private final CANSparkMax topShooterMotor;

  private double distance = 3;
  private double distanceOffset;
  private double velocity;
  private double angle;
  private double height;

  private double midpointY = 1.974;
  private double midpointX = 0.196;

  private double elevatorPivotHeight = Units.inchesToMeters(23.75);
  private double elevatorPivotLength = Units.inchesToMeters(12);
  private double shooterFromElevator = Units.inchesToMeters(7);
  private double pivotAngle = 0;
  //relative to ground
  private double flywheelOffset = Units.degreesToRadians(30); //60 degrees tilted up
  private double elevatorXOffset = Units.inchesToMeters(-1); //positive further back negative further forward

  private double wheelCircMeters = 0.316484;

  private TalonFX m_rightMotor = new TalonFX(0);
  private TalonFX m_leftMotor = new TalonFX(1);

  /** Creates a new IntakeSubsystem. */
  public ShooterSubsystem() {
    //constructor go brrrrrrr
    var shooterConfig = new TalonFXConfiguration();
    shooterConfig.Feedback.SensorToMechanismRatio = wheelCircMeters / 60;

    m_leftMotor.getConfigurator().apply(shooterConfig);
    m_rightMotor.getConfigurator().apply(shooterConfig);
  }

  @Override
  public void periodic() {

  }

  public double findVelocity(double x){
    return Math.sqrt((((
                    (-1*(9.8*Math.pow(-x, 2)))
              /((midpointY + height)/(-x*Math.tan(angle))))
                /Math.pow(Math.cos(angle), 2)))
                                /2);
  }

  public double findTrajectoryPoint(double x){
    return((distance + x)*Math.tan(angle)-((9.8*Math.pow((distance + x), 2)) / (2*Math.pow(findVelocity(distance), 2) * Math.pow(Math.cos(angle), 2))));
  }

  public double findBestAngle(){
    double add = 0.1;
    int searchLength = 100;
    if(findTrajectoryPoint(-midpointX) < midpointX){
      add *= -1;
    }

    double startAngle = Units.radiansToDegrees(Math.atan((midpointY-height)/(distance - midpointX)))+6;
    double[] angles = new double[searchLength];

    for(int i = 0; i < searchLength; i++){
      angles[i] = startAngle+(add*i);
    }

    //a high number aka mechanical advantage
    double closestError = 6328;
    int angleIndex = 0;

    double originalDistance = distance;

    for(int i = 0; i < searchLength; i++){
      angle = Units.degreesToRadians(angles[i]);
      height = elevatorPivotHeight-(elevatorPivotLength*Math.cos(angle - flywheelOffset)) + (shooterFromElevator*Math.sin(angle));

      double xOffset = (elevatorPivotLength*Math.sin(angle - flywheelOffset)) + (shooterFromElevator*Math.sin(Units.degreesToRadians(90) - angle));
      distance = originalDistance - xOffset + elevatorXOffset;
      double error = Math.abs(findTrajectoryPoint(-midpointX) - (midpointY - height));

      if (error < closestError){
        closestError = error;
        angleIndex = i;
      }
    }
    return angles[angleIndex];
  }

  public void calculateShooter(){
    angle = findBestAngle();

    double originalDistance = distance;
    double xOffset = (elevatorPivotLength*Math.sin(angle - flywheelOffset)) + (shooterFromElevator*Math.sin(Units.degreesToRadians(90) - angle));
    distance = originalDistance - xOffset + elevatorXOffset;
    velocity = findVelocity(distance);

    angle = Units.degreesToRadians(angle);
    height = elevatorPivotHeight-(elevatorPivotLength*Math.cos(angle - flywheelOffset)) + (shooterFromElevator*Math.sin(angle));
    pivotAngle = angle - Units.radiansToDegrees(flywheelOffset);
  }

  public double getPivotAngle(){
    return Units.radiansToDegrees(pivotAngle);
  }

  public void setFlywheelSpeed(double speed){
    m_leftMotor.set(speed);
    m_rightMotor.set(-speed);
  }
}
