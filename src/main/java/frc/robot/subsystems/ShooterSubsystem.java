// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private double distance = 4.5;
  private double velocity;
  private double angle;
  private double height = 0.7;

  private double midpointY = 1.974;
  private double midpointX = 0.196;


  /** Creates a new IntakeSubsystem. */
  public ShooterSubsystem() {
    //constructor go brrrrrrr
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

  public double  findTrajectoryPoint(double x){
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

    for(int i = 0; i < searchLength; i++){
      angle = Units.degreesToRadians(angles[i]);
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
    velocity = findVelocity(distance);
  }
}