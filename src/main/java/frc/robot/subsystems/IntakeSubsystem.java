// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax m_topRoller = new CANSparkMax(15, MotorType.kBrushless);
  private final CANSparkMax m_bottomRoller = new CANSparkMax(16, MotorType.kBrushless);

  private static double distance = 4.5;
  private static double velocity;
  private static double angle = Units.degreesToRadians(54.8);
  private static double height = 0.7;

  private static double midpointY = 1.974;
  private static double midpointX = 0.196;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    m_topRoller.setInverted(true);
    m_bottomRoller.setInverted(true);
  }

  public void setRollerSpeed(double speed) {
    m_topRoller.set(speed);
    m_bottomRoller.set(speed);
  }

  public void stop() {
    setRollerSpeed(0.0);
  }

  @Override
  public void periodic() {

  }
  public static double findVelocity(double x){
    return Math.sqrt((((
                    (-1*(9.8*Math.pow(-x, 2)))
              /((midpointY + height)/(-x*Math.tan(angle))))
                /Math.pow(Math.cos(angle), 2)))
                                /2);
  }
  public static double  findTrajectoryPoint(double x){
    System.out.println("angle in trajectory: " + angle);
    return((distance + x)*Math.tan(angle)-((9.8*Math.pow((distance + x), 2)) / (2*Math.pow(findVelocity(distance), 2) * Math.pow(Math.cos(angle), 2))));
  }
  public static double findBestAngle(){
    double add = 0.1;
    int searchLength = 100;
    if(findTrajectoryPoint(-midpointX) < midpointX){
      add *= -1;
    }

    double startAngle = Units.radiansToDegrees(Math.atan((midpointY-height)/(distance - midpointX)))+6;

    System.out.println("add: " + add);
    System.out.println("starting angle: " + startAngle);

    double[] angles = new double[searchLength];

    for(int i = 0; i < searchLength; i++){
      angles[i] = startAngle+(add*i);
    }

    double closestError = 360;
    int angleIndex = 0;
    for(int i = 0; i < searchLength; i++){
      angle = Units.degreesToRadians(angles[i]);
      double error = Math.abs(findTrajectoryPoint(-midpointX) - (midpointY - height));
      System.out.println("error: " + error + " angle: " + Units.radiansToDegrees(angle));
      System.out.println("\n Trajectory Point: " + Math.abs(findTrajectoryPoint(-midpointX)) + "\n");
      if (error < closestError){
        closestError = error;
        angleIndex = i;
      }
    }
    return angles[angleIndex];
  }
  public static void main(String[] args){
    System.out.println("test");
    //System.out.println(findTrajectoryPoint(-midpointX));
    //System.out.println(findVelocity(distance));
    
    double bestAngle = findBestAngle();
    System.out.println("best angle: " + bestAngle);
    
    angle = Units.degreesToRadians(bestAngle);
    
    System.out.println("velocity m/s: " + findVelocity(distance));
  }
}