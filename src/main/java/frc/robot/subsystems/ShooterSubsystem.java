// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
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

  private double midpointY = 1.974; //+ Units.inchesToMeters(3);
  private double midpointX = 0.196;

  private double elevatorPivotHeight = Units.inchesToMeters(23.75);
  private double elevatorPivotLength = Units.inchesToMeters(12);
  private double shooterFromElevator = Units.inchesToMeters(7);
  private double pivotAngle = 0;
  //relative to ground
  private double flywheelOffset = Units.degreesToRadians(30); //60 degrees tilted up
  private double elevatorXOffset = Units.inchesToMeters(-1); //positive further back negative further forward

  private double wheelCircMeters = Math.PI*Units.inchesToMeters(4);

  //TODO check rev client and add new device ids
  private final CANSparkMax m_topMotor = new CANSparkMax(13, MotorType.kBrushless);
  private final CANSparkMax m_bottomMotor = new CANSparkMax(14, MotorType.kBrushless);

  private PIDController m_topController = new PIDController(0.00023*0.9, 0, 0.1);
  private PIDController m_bottomController = new PIDController(0.00027*0.8, 0, 0.1);

  /** Creates a new IntakeSubsystem. */
  public ShooterSubsystem() {
    //constructor go brrrrrrr
    // var shooterConfig = new TalonFXConfiguration();
    // shooterConfig.Feedback.SensorToMechanismRatio = wheelCircMeters / 60;

    // m_leftMotor.getConfigurator().apply(shooterConfig);
    // m_rightMotor.getConfigurator().apply(shooterConfig);

    m_topMotor.restoreFactoryDefaults();
    m_bottomMotor.restoreFactoryDefaults();

    m_bottomMotor.setInverted(true);
    m_topMotor.setInverted(false);
    m_bottomMotor.getEncoder().setVelocityConversionFactor(24/18);

    m_topMotor.setSmartCurrentLimit(40);
    m_bottomMotor.setSmartCurrentLimit(40);

    m_topMotor.burnFlash();
    m_bottomMotor.burnFlash();
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Top Veloctiy", m_topMotor.getEncoder().getVelocity());
    Logger.recordOutput("Bottom Velocity", m_bottomMotor.getEncoder().getVelocity());
    Logger.recordOutput("Velocity Setpoint", RPMFromMPS(findVelocity(distance)));
  }

  public double RPMFromMPS(double mps){
    return mps * 60 / wheelCircMeters;
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

  public void setFlywheelSpeed(){
    distance = 4.6;
    angle = Units.degreesToRadians(30);
    height = Units.inchesToMeters(13.5);

    m_topController.setSetpoint(RPMFromMPS(findVelocity(distance)));
    m_bottomController.setSetpoint(RPMFromMPS(findVelocity(distance)));

    m_bottomMotor.set(m_bottomController.calculate(m_bottomMotor.getEncoder().getVelocity())*0.95);
    m_topMotor.set(m_topController.calculate(m_topMotor.getEncoder().getVelocity()));
  }

  public void stop(){
    m_topMotor.set(0);
    m_bottomMotor.set(0);
  }
}
