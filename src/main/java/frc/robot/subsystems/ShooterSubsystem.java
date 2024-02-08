// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private final CANSparkMax topShooterMotor;

  private double m_velocity;
  private double m_angle;
  private double m_height;

  private double m_pivotAngle;

  private TalonFX m_leftFlywheel = new TalonFX(Constants.LEFT_FLYWHEEL);
  private TalonFX m_rightFlywheel = new TalonFX(Constants.RIGHT_FLYWHEEL);
  private CANSparkMax m_indexer = new CANSparkMax(Constants.INDEXER, MotorType.kBrushless);

  /** Creates a new IntakeSubsystem. */
  public ShooterSubsystem() {
    //constructor go brrrrrrr
    var leftConfig = new TalonFXConfiguration();
    //set inverted here
    leftConfig.Feedback.SensorToMechanismRatio = Constants.FLYWHEEL_CIRCUMFERENCE/60;
    leftConfig.CurrentLimits.SupplyCurrentLimit = 40;
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    var leftSlot0Config = leftConfig.Slot0;
    leftSlot0Config.kS = 0.25; // Add 0.25 V output to overcome static friction
    leftSlot0Config.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    leftSlot0Config.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    leftSlot0Config.kP = 0.11; // An error of 1 rps results in 0.11 V output
    leftSlot0Config.kI = 0; // no output for integrated error
    leftSlot0Config.kD = 0; // no output for error derivative

    var leftMotionMagicConfig = leftConfig.MotionMagic;
    leftMotionMagicConfig.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
    leftMotionMagicConfig.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)

    m_leftFlywheel.getConfigurator().apply(leftConfig);

    var rightConfig = new TalonFXConfiguration();
    //set inverted here
    rightConfig.Feedback.SensorToMechanismRatio = Constants.FLYWHEEL_CIRCUMFERENCE/60;
    rightConfig.CurrentLimits.SupplyCurrentLimit = 40;
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    var rightSlot0Config = leftConfig.Slot0;
    rightSlot0Config.kS = 0.25; // Add 0.25 V output to overcome static friction
    rightSlot0Config.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    rightSlot0Config.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    rightSlot0Config.kP = 0.11; // An error of 1 rps results in 0.11 V output
    rightSlot0Config.kI = 0; // no output for integrated error
    rightSlot0Config.kD = 0; // no output for error derivative

    var rightMotionMagicConfig = leftConfig.MotionMagic;
    rightMotionMagicConfig.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
    rightMotionMagicConfig.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)

    m_rightFlywheel.getConfigurator().apply(rightConfig);

    m_indexer.restoreFactoryDefaults();
    //set inverted here
    m_indexer.setSmartCurrentLimit(20);
    m_indexer.setIdleMode(IdleMode.kBrake);
    m_indexer.burnFlash();
  }

  @Override
  public void periodic() {

  }

  public double RPMFromMPS(double mps){
    return mps * 60 / Constants.FLYWHEEL_CIRCUMFERENCE;
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

  public void calculateShooter(double distance){
    m_angle = findBestAngle(distance);

    double originalDistance = distance;
    double xOffset = (Constants.ELEVATOR_PIVOT_LENGTH*Math.sin(m_angle - Constants.FLYWHEEL_OFFSET)) + (Constants.SHOOTER_FROM_ELEVATOR*Math.sin(Units.degreesToRadians(90) - m_angle));
    distance = originalDistance - xOffset + Constants.ELEVATOR_X_OFFSET;
    m_velocity = findVelocity(distance);

    m_angle = Units.degreesToRadians(m_angle);
    m_height = Constants.ELEVATOR_PIVOT_HEIGHT-(Constants.ELEVATOR_PIVOT_LENGTH*Math.cos(m_angle - Constants.FLYWHEEL_OFFSET)) + (Constants.SHOOTER_FROM_ELEVATOR*Math.sin(m_angle));
    m_pivotAngle = m_angle - Units.radiansToDegrees(Constants.FLYWHEEL_OFFSET);
  }

  public double getPivotAngle(){
    return Units.radiansToDegrees(m_pivotAngle);
  }

  public void setFlywheelSpeed(double velocity){
    final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);
    m_rightFlywheel.setControl(m_request.withVelocity(velocity));
    m_leftFlywheel.setControl(m_request.withVelocity(velocity));
  }
  public void stopFlywheel(){
    final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);
    m_rightFlywheel.setControl(m_request.withVelocity(0));
    m_leftFlywheel.setControl(m_request.withVelocity(0));
  }

  public void setIndexerSpeed(double speed){
    m_indexer.set(speed);
  }
  public void stopIndexer(){
    m_indexer.set(0);
  }
}
