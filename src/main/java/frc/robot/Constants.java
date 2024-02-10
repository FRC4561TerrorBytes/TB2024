// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

  import com.ctre.phoenix6.signals.InvertedValue;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final int PIGEON_ID = 20;

  public static final int FALCON_500_MAX_RPM = 6380;
  public static final int CTRE_TALONFX_ENCODER_TICKS_PER_ROTATION = 2048;
  public static final int NEO_MAX_RPM = 5676;
  public static final int NEO_TICKS_PER_ROTATION = 4096;

  public static final double MAX_VOLTAGE = 12.0;
  public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.53;
  public static final double DRIVETRAIN_WHEELBASE_METERS = 0.53;

  public static final double MAX_VELOCITY_METERS_PER_SECOND = FALCON_500_MAX_RPM / 60.0 *
        (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0) *
        0.10033 * Math.PI;

  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
      Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
          DRIVETRAIN_WHEELBASE_METERS / 2.0);

  public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
      // Front left
      new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
          Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Front right
      new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
          -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back left
      new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
          Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back right
      new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
          -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0));
   //MODULE 0
  public static final int FRONT_LEFT_DRIVE_MOTOR = 1; //5 for fulcrum
  public static final int FRONT_LEFT_STEER_MOTOR = 2; //6 for fulcrum
  public static final int FRONT_LEFT_STEER_ENCODER = 21; //23 for fulcrum
  public static final InvertedValue FRONT_LEFT_DRIVE_MOTOR_INVERTED = InvertedValue.Clockwise_Positive; //counter clockwise for fulcrum
  public static final boolean FRONT_LEFT_TURN_MOTOR_INVERTED = true;
  public static final double FRONT_LEFT_STEER_OFFSET = Units.degreesToRadians(102.4711046);//0.2393010029 Fulcrum drivebase

  //MODULE 1
  public static final int FRONT_RIGHT_DRIVE_MOTOR = 7; //3 for fulcrum
  public static final int FRONT_RIGHT_STEER_MOTOR = 8; //4 for fulcrum
  public static final int FRONT_RIGHT_STEER_ENCODER = 24; //22 for fulcrum
  public static final InvertedValue FRONT_RIGHT_DRIVE_MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive;
  public static final boolean FRONT_RIGHT_TURN_MOTOR_INVERTED = true;
  public static final double FRONT_RIGHT_STEER_OFFSET = Units.degreesToRadians(76);//1.73646625; Fulcrum drivebase
  //MODULE 2
  public static final int BACK_LEFT_DRIVE_MOTOR = 3; //7 for fulcrum
  public static final int BACK_LEFT_STEER_MOTOR = 4; //8 for fulcrum
  public static final int BACK_LEFT_STEER_ENCODER = 22; //24 for fulcum
  public static final InvertedValue BACK_LEFT_DRIVE_MOTOR_INVERTED = InvertedValue.Clockwise_Positive; //counter clockwise for fulcrum
  public static final boolean BACK_LEFT_TURN_MOTOR_INVERTED = true;
  public static final double BACK_LEFT_STEER_OFFSET = Units.degreesToRadians(-140);//-2.5632818; Fulcrum drivebase

  //MODULE 3
  public static final int BACK_RIGHT_DRIVE_MOTOR = 5; //1 for fulcrum
  public static final int BACK_RIGHT_STEER_MOTOR = 6; //2 for fulcrum
  public static final int BACK_RIGHT_STEER_ENCODER = 23; //21 for fulcrum
  public static final InvertedValue BACK_RIGHT_DRIVE_MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive; //clockwise for fulcrum
  public static final boolean BACK_RIGHT_TURN_MOTOR_INVERTED = true;
  public static final double BACK_RIGHT_STEER_OFFSET = Units.degreesToRadians(140);//-2.1184274; Fulcrum drivebase

  public static final double DRIVE_CURRENT_LIMIT = 80.0;
  public static final int TURN_CURRENT_LIMIT = 40;
  public static final double DRIVE_CURRENT_THRESHOLD = 120.0;
  public static final double DRIVE_CURRENT_TIME_THRESHOLD = 0.1; // seconds
  // current limits for drivetrain turn and steer motors
  // (Amps)

  public static final double TURN_MOTOR_KP = 1.0;
  public static final double TURN_MOTOR_KI = 0.0;
  public static final double TURN_MOTOR_KD = 0.1;
  public static final double TURN_MOTOR_MECHANICAL_EFFICIENCY = 1.0;
  public static final double TURN_MOTOR_TOLERANCE = Math.PI / 180;
  public static final double TURN_MOTOR_LOWER_LIMIT = 0.0;
  public static final double TURN_MOTOR_UPPER_LIMIT = 0.0;
  public static final boolean TURN_ENABLE_SOFT_LIMITS = false;
  public static final double TURN_MOTOR_CONVERSION_FACTOR = 2 * Math.PI
      * (14.0 / 50.0) * (10.0 / 60.0);
  public static final double DRIVE_MOTOR_CONVERSION_FACTOR = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0)
      * 0.10033 * Math.PI
      / CTRE_TALONFX_ENCODER_TICKS_PER_ROTATION;

  // fyi: offset right means lateral offset from center
  public static final double LEFT_CAMERA_OFFSET_RIGHT = Units.inchesToMeters(11.75);
  public static final double RIGHT_CAMERA_OFFSET_RIGHT = Units.inchesToMeters(-11.75);
  public static final double LEFT_CAMERA_OFFSET_BACK = Units.inchesToMeters(20);
  public static final double RIGHT_CAMERA_OFFSET_BACK = Units.inchesToMeters(20);
  public static final double VISION_ROTATION_SCALING = 0.1;
  public static final double VISION_LATERAL_SCALING = 2;

  public static final double VISION_ROTATION_FLOOR_CLAMP = 2;
  public static final double VISION_ROTATION_CEILING_CLAMP = 50;
  public static final double VISION_ROTATION_DEADBAND = 6;
  public static final double VISION_ROTATION_TOLERANCE = 2;

  public static final double VISION_LATERAL_FLOOR_CLAMP = 0.2;
  public static final double VISION_LATERAL_CEILING_CLAMP = 0.5;
  public static final double VISION_LATERAL_DEADBAND = 0.1;
  public static final double VISION_LATERAL_TOLERANCE = 0.05;

  public static final double VISION_END_DISTANCE = Units.inchesToMeters(9);
  public static final double VISION_FORWARD_FLOOR_CLAMP = 0.2;
  public static final double VISION_FORWARD_CEILING_CLAMP = 1;

//   public static final SparkPIDConfig TURN_MOTOR_CONFIG = new SparkPIDConfig(
//       false,
//       NEO_MAX_RPM,
//       TURN_MOTOR_KP,
//       TURN_MOTOR_KI,
//       TURN_MOTOR_KD,
//       TURN_MOTOR_MECHANICAL_EFFICIENCY,
//       TURN_MOTOR_TOLERANCE,
//       TURN_MOTOR_LOWER_LIMIT,
//       TURN_MOTOR_UPPER_LIMIT,
//       TURN_ENABLE_SOFT_LIMITS,
//       NEO_MAX_RPM,
//       NEO_MAX_RPM,
//       TURN_MOTOR_CONVERSION_FACTOR);

  public static final double ELEVATOR_MOTOR_GEAR_RATIO = 16.15;
  public static final double ELEVATOR_RATIO = 11.0/720.0;

  public static final double SHOOTER_MOTOR_GEAR_RATIO = 1.0;
    
  public static final double INDEXER_MOTOR_GEAR_RATIO = 1.0;

  public static final double INTAKE_MOTOR_GEAR_RATIO = 1.0;
  public static final double BAR_MOTOR_GEAR_RATIO = 1.0;

  public static final int ROLLER_MOTOR = 12;

  public static final int LEFT_FLYWHEEL = 99;
  public static final int RIGHT_FLYWHEEL = 99;
  public static final int INDEXER = 99;

  public static final double TARGET_Y = 1.974; //+ Units.inchesToMeters(3);
  public static final double TARGET_X = 0.196;

  public static final double ELEVATOR_PIVOT_HEIGHT = Units.inchesToMeters(23.75);
  public static final double ELEVATOR_PIVOT_LENGTH = Units.inchesToMeters(12);
  public static final double SHOOTER_FROM_ELEVATOR = Units.inchesToMeters(7);

  //relative to ground
  public static final double FLYWHEEL_OFFSET = Units.degreesToRadians(30); //30 degrees tilted up
  public static final double ELEVATOR_X_OFFSET = Units.inchesToMeters(-1); //positive further back negative further forward

  public static final double FLYWHEEL_CIRCUMFERENCE = Math.PI*Units.inchesToMeters(4);

  public static final double INDEXER_FEED_SPEED = 0.05;

  public static final double AUTO_X_KP = 1.0;
  public static final double AUTO_X_KI = 0.0;
  public static final double AUTO_X_KD = 0.0;

  public static final double AUTO_Y_KP = 1.0;
  public static final double AUTO_Y_KI = 0.0;
  public static final double AUTO_Y_KD = 0.0;

  public static final double AUTO_THETA_KP = 1.0;
  public static final double AUTO_THETA_KI = 0.0;
  public static final double AUTO_THETA_KD = 0.0;

  public static final int FRONT_INTAKE = 15;
  public static final int BACK_INTAKE = 16;
  public static final int INTAKE_ROTATOR = 17;
  public static final int ROTATOR_ROLLER = 22;

  public static final Mode currentMode = Mode.SIM;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY

    }

}

