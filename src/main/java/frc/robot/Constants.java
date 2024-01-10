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

  public static final int FRONT_LEFT_DRIVE_MOTOR = 5;
  public static final int FRONT_LEFT_STEER_MOTOR = 6;
  public static final int FRONT_LEFT_STEER_ENCODER = 23;
  public static final boolean FRONT_LEFT_DRIVE_MOTOR_INVERTED = false;
  public static final boolean FRONT_LEFT_TURN_MOTOR_INVERTED = true;
  public static final double FRONT_LEFT_STEER_OFFSET = 13;

  public static final int FRONT_RIGHT_DRIVE_MOTOR = 3;
  public static final int FRONT_RIGHT_STEER_MOTOR = 4;
  public static final int FRONT_RIGHT_STEER_ENCODER = 22;
  public static final boolean FRONT_RIGHT_DRIVE_MOTOR_INVERTED = false;
  public static final boolean FRONT_RIGHT_TURN_MOTOR_INVERTED = true;
  public static final double FRONT_RIGHT_STEER_OFFSET = 278.7;

  public static final int BACK_LEFT_DRIVE_MOTOR = 7;
  public static final int BACK_LEFT_STEER_MOTOR = 8;
  public static final int BACK_LEFT_STEER_ENCODER = 24;
  public static final boolean BACK_LEFT_DRIVE_MOTOR_INVERTED = true;
  public static final boolean BACK_LEFT_TURN_MOTOR_INVERTED = true;
  public static final double BACK_LEFT_STEER_OFFSET = 27.86;

  public static final int BACK_RIGHT_DRIVE_MOTOR = 1;
  public static final int BACK_RIGHT_STEER_MOTOR = 2;
  public static final int BACK_RIGHT_STEER_ENCODER = 21;
  public static final boolean BACK_RIGHT_DRIVE_MOTOR_INVERTED = false;
  public static final boolean BACK_RIGHT_TURN_MOTOR_INVERTED = true;
  public static final double BACK_RIGHT_STEER_OFFSET = 60.73;

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

  public static final int ELBOW_MOTOR = 11;
  public static final int SHOULDER_MOTOR = 10;
  public static final int WRIST_MOTOR = 13;

  public static final double ELBOW_ROTATIONS_PER_DEGREE = 225.0 / 360.0;
  public static final double ELBOW_MAX_VOLTAGE_FF = 2.0;
  public static final double ELBOW_PROPORTIONAL_GAIN_SLOT_0 = 1.5; //UP
  public static final double ELBOW_PROPORTIONAL_GAIN_SLOT_1 = 0.025; //DOWN
  public static final double ELBOW_DERIVATIVE_GAIN_SLOT_0 = 0.0009;
  public static final double ELBOW_DERIVATIVE_GAIN_SLOT_1 = 0.0001;
  public static final double ELBOW_INTEGRAL_GAIN = 0.000015;
  public static final double ELBOW_IZONE = 5.0;
  public static final double ELBOW_TOLERANCE = ELBOW_ROTATIONS_PER_DEGREE / 2.0;
  public static final double ELBOW_NUDGE_DEGREES = 3.0;
  public static final int ELBOW_ENCODER_COUNT_PER_REV = 8192;
  public static final double ELBOW_POSITION_OFFSET = 9;

  public static final double SHOULDER_ROTATIONS_PER_DEGREE = 500.0 / 360.0;
  // needs fixing; moves to fast and with to much force.
  public static final double SHOULDER_MAX_VOLTAGE_FF = 0.1;
  public static final double SHOULDER_PROPORTIONAL_GAIN_SLOT_0 = 0.001; // Scoring (May change to 0.022)
  public static final double SHOULDER_PROPORTIONAL_GAIN_SLOT_1 = 0.025; // Returning from score (May change to 0.033)
  public static final double SHOULDER_INTEGRAL_GAIN_SLOT_0 = 0.00005;
  public static final double SHOULDER_DERIVATIVE_GAIN = 0.06;
  public static final double SHOULDER_TOLERANCE = SHOULDER_ROTATIONS_PER_DEGREE / 2.0;
  public static final double SHOULDER_NUDGE_DEGREES = 3.0; //Placeholder value, this will 
  public static final int SHOULDER_ENCODER_COUNT_PER_REV = 8192;
  public static final double SHOULDER_ENCODER_ROTATIONS_PER_DEGREE = 1.89;
  public static final double SHOULDER_POSITION_OFFSET = 3.5;
  
  public static final double WRIST_ROTATIONS_PER_DEGREE = 60.0 / 360.0;
  //TODO maybe tune this?
  public static final double WRIST_MAX_VOLTAGE_FF = 0.0;
  //TODO this is really snappy, but it might be too high.
  public static final double WRIST_PROPORTIONAL_GAIN = 0.01;
  public static final double WRIST_TOLERANCE = WRIST_ROTATIONS_PER_DEGREE / 2.0;
  public static final double WRIST_NUDGE_DEGREES = 4.0;
  public static final int WRIST_ENCODER_COUNTS_PER_REV = 8192;
  public static final double WRIST_ENCODER_OFFSET = -7.5;
  public static final double WRIST_ENCODER_ZERO_THRESHOLD = 3.0; // random test value

  public static final int ROLLER_MOTOR = 12;

  public static final double INTAKE_SPEED = 0.4;
  public static final double INTAKE_HOLD_SPEED = 0.05;

  public static final double SCORE_SPEED_CONE = -0.35;
  public static final double SCORE_SPEED_CUBE = -0.25;

  public static final double AUTO_X_KP = 1.0;
  public static final double AUTO_X_KI = 0.0;
  public static final double AUTO_X_KD = 0.0;

  public static final double AUTO_Y_KP = 1.0;
  public static final double AUTO_Y_KI = 0.0;
  public static final double AUTO_Y_KD = 0.0;

  public static final double AUTO_THETA_KP = 1.0;
  public static final double AUTO_THETA_KI = 0.0;
  public static final double AUTO_THETA_KD = 0.0;

  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY

    }

}

