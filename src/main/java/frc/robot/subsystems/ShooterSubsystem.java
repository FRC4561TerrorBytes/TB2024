// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private static double distance = 3;
  private double distanceOffset;
  private static double velocity;
  private static double angle;
  private static double height;

  private static double midpointY = 1.974;
  private static double midpointX = 0.196;

  private static double elevatorPivotHeight = Units.inchesToMeters(23.75);
  private static double elevatorPivotLength = Units.inchesToMeters(12);
  private static double shooterFromElevator = Units.inchesToMeters(7);
  private static double pivotAngle = 0;
  //relative to ground
  private static double flywheelOffset = Units.degreesToRadians(30); //60 degrees tilted up
  private static double elevatorXOffset = Units.inchesToMeters(-1); //positive further back negative further forward

  private double wheelCircMeters = 0.316484;


  /** Creates a new IntakeSubsystem. */
  public ShooterSubsystem() {
    //constructor go brrrrrrr
  }

  @Override
  public void periodic() {

  }

  public double rpmFromMPS(double mps) {
    return mps * 60 / wheelCircMeters;
  }

  public static double findVelocity(double x){
    return Math.sqrt((((
                    (-1*(9.8*Math.pow(-x, 2)))
              /((midpointY + height)/(-x*Math.tan(angle))))
                /Math.pow(Math.cos(angle), 2)))
                                /2);
  }

  public static double findTrajectoryPoint(double x){
    return((distance + x)*Math.tan(angle)-((9.8*Math.pow((distance + x), 2)) / (2*Math.pow(findVelocity(distance), 2) * Math.pow(Math.cos(angle), 2))));
  }

  public static double findBestAngle(){
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
      //System.out.println("\nheight offset: " + height);
      //System.out.println("Angle: " + Units.radiansToDegrees(angle));

      double xOffset = (elevatorPivotLength*Math.sin(angle - flywheelOffset)) + (shooterFromElevator*Math.sin(Units.degreesToRadians(90) - angle));
      //System.out.println("distance offset: " + xOffset + "\n");
      distance = originalDistance - xOffset + elevatorXOffset;
      double error = Math.abs(findTrajectoryPoint(-midpointX) - (midpointY - height));

      if (error < closestError){
        closestError = error;
        angleIndex = i;
      }
    }
    return angles[angleIndex];
  }

  public static void calculateShooter(){
    angle = findBestAngle();
    velocity = findVelocity(distance);

    
    System.out.println("best angle: " + angle);

    angle = Units.degreesToRadians(angle);
    // height = elevatorPivotHeight-(elevatorPivotLength*Math.cos(angle - flywheelOffset)) + (shooterFromElevator*Math.sin(angle));
    // double originalDistance = distance;
    double xOffset = (elevatorPivotLength*Math.sin(angle - flywheelOffset)) + (shooterFromElevator*Math.sin(Units.degreesToRadians(90) - angle));
    height = elevatorPivotHeight-(elevatorPivotLength*Math.cos(angle - flywheelOffset)) + (shooterFromElevator*Math.sin(angle));
    // distance = originalDistance - xOffset + elevatorXOffset;
    System.out.println("x offset: " + xOffset);
    System.out.println("height: " + height);
    pivotAngle = angle - Units.radiansToDegrees(flywheelOffset);
    System.out.println("pivot angle: " + pivotAngle);
  }

  public static void main(String[] args){
    calculateShooter();
  }
}