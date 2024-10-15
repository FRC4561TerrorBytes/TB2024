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
        
        public static final double DRIVE_DEADBAND = 0.125;

        public static final int KRAKEN_X60_MAX_RPM = 6000;
        public static final int CTRE_TALONFX_ENCODER_TICKS_PER_ROTATION = 2048;
        public static final int NEO_MAX_RPM = 5676;
        public static final int NEO_TICKS_PER_ROTATION = 4096;

        public static final double MAX_VOLTAGE = 12.0;
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(26.0);
        public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(26.0);

        public static final double MAX_VELOCITY_METERS_PER_SECOND = KRAKEN_X60_MAX_RPM / 60.0 *
                        (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0) *
                        0.10033 * Math.PI;

        public static final double DRIVETRAIN_RADIUS = Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                        DRIVETRAIN_WHEELBASE_METERS / 2.0);

        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND
                        / DRIVETRAIN_RADIUS;

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

        public static final Translation2d[] MODULE_TRANSLATIONS = new Translation2d[] {
                        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
        };

        // MODULE 0
        public static final int FRONT_LEFT_DRIVE_MOTOR = 1; // 5 for fulcrum
        public static final int FRONT_LEFT_STEER_MOTOR = 2; // 6 for fulcrum
        public static final int FRONT_LEFT_STEER_ENCODER = 21; // 23 for fulcrum
        public static final InvertedValue FRONT_LEFT_DRIVE_MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive; // counter
                                                                                                                     // clockwise
                                                                                                                     // for
                                                                                                                     // fulcrum
        public static final boolean FRONT_LEFT_TURN_MOTOR_INVERTED = true;
        public static final double FRONT_LEFT_STEER_OFFSET = 1.77465;// 0.2393010029 Fulcrum
                                                                     // drivebase

        // MODULE 1
        public static final int FRONT_RIGHT_DRIVE_MOTOR = 7; // 3 for fulcrum
        public static final int FRONT_RIGHT_STEER_MOTOR = 8; // 4 for fulcrum
        public static final int FRONT_RIGHT_STEER_ENCODER = 24; // 22 for fulcrum
        public static final InvertedValue FRONT_RIGHT_DRIVE_MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive;
        public static final boolean FRONT_RIGHT_TURN_MOTOR_INVERTED = true;
        public static final double FRONT_RIGHT_STEER_OFFSET = -1.82381;// 1.73646625; Fulcrum drivebase
        // MODULE 2
        public static final int BACK_LEFT_DRIVE_MOTOR = 3; // 7 for fulcrum
        public static final int BACK_LEFT_STEER_MOTOR = 4; // 8 for fulcrum
        public static final int BACK_LEFT_STEER_ENCODER = 22; // 24 for fulcum
        public static final InvertedValue BACK_LEFT_DRIVE_MOTOR_INVERTED = InvertedValue.Clockwise_Positive; // counter
                                                                                                             // clockwise
                                                                                                             // for
                                                                                                             // fulcrum
        public static final boolean BACK_LEFT_TURN_MOTOR_INVERTED = true;
        public static final double BACK_LEFT_STEER_OFFSET = 0.690461;// -2.5632818; Fulcrum drivebase

        // MODULE 3
        public static final int BACK_RIGHT_DRIVE_MOTOR = 5; // 1 for fulcrum
        public static final int BACK_RIGHT_STEER_MOTOR = 6; // 2 for fulcrum
        public static final int BACK_RIGHT_STEER_ENCODER = 23; // 21 for fulcrum
        public static final InvertedValue BACK_RIGHT_DRIVE_MOTOR_INVERTED = InvertedValue.Clockwise_Positive; // clockwise
                                                                                                              // for //
                                                                                                              // fulcrum
        public static final boolean BACK_RIGHT_TURN_MOTOR_INVERTED = true;
        public static final double BACK_RIGHT_STEER_OFFSET = 2.4541988;// -2.1184274; Fulcrum drivebase

        public static final double ARM_CURRENT_LIMIT = 40.0;

        public static final double DRIVE_CURRENT_LIMIT = 50.0;
        public static final int TURN_CURRENT_LIMIT = 20;
        public static final double DRIVE_STATOR_CURRENT_LIMIT = 125;
        public static final double DRIVE_CURRENT_THRESHOLD = 120.0;
        public static final double DRIVE_CURRENT_TIME_THRESHOLD = 0.1; // seconds
        // current limits for drivetrain turn and steer motors
        // (Amps)

        public static final double SHOOTER_SUPPLY_CURRENT_LIMIT = 30.0;
        public static final double SHOOTER_STATOR_CURRENT_LIMIT = 100.0;

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

        // public static final SparkPIDConfig TURN_MOTOR_CONFIG = new SparkPIDConfig(
        // false,
        // NEO_MAX_RPM,
        // TURN_MOTOR_KP,
        // TURN_MOTOR_KI,
        // TURN_MOTOR_KD,
        // TURN_MOTOR_MECHANICAL_EFFICIENCY,
        // TURN_MOTOR_TOLERANCE,
        // TURN_MOTOR_LOWER_LIMIT,
        // TURN_MOTOR_UPPER_LIMIT,
        // TURN_ENABLE_SOFT_LIMITS,
        // NEO_MAX_RPM,
        // NEO_MAX_RPM,
        // TURN_MOTOR_CONVERSION_FACTOR);

        public static final double ELEVATOR_MOTOR_GEAR_RATIO = 16.15;
        public static final double ELEVATOR_RATIO = 11.0 / 720.0;

        // Needs Calculation
        // To calculate kS find smallest volatage to move
        // To calculate kG, and kV use Recalc.
        public static final double ELEVATOR_STATIC_GAIN = 0.1;
        public static final double ELEVATOR_GRAVITY_GAIN = 0.14;
        public static final double ELEVATOR_VELOCITY_GAIN = 46.63;
        public static final double ELEVATOR_ACCELERATION_GAIN = 0.02;

        public static enum rgbValues {
                HIGHTIDE_BLUE(0, 182, 174),
                NOTE_INTAKEN(255, 40, 3),
                PURPLE_SHOOT(200, 5, 200),
                BLUE_254(20, 65, 254),
                FUNNY_COLOR(176, 11, 105),
                PINK(255, 10, 10),
                GREEN(0, 200, 0),
                BLANK(0, 0, 0);

                public int r;
                public int g;
                public int b;

                private rgbValues(int r, int g, int b) {
                        this.r = r;
                        this.g = g;
                        this.b = b;
                }
        }

        public static final double SHOOTER_MOTOR_GEAR_RATIO = 1.0;

        public static final double INDEXER_MOTOR_GEAR_RATIO = 1.0;

        public static final double INTAKE_MOTOR_GEAR_RATIO = 1.0;
        public static final double BAR_MOTOR_GEAR_RATIO = 1.0;

        public static final double INTAKE_SPEED = 1.0;
        public static final double INTAKE_LOW_POSITION = 110;
        public static final double INTAKE_HIGH_POSITION = 240;

        public static final int LEFT_FLYWHEEL = 51;
        public static final int RIGHT_FLYWHEEL = 52;
        public static final int INDEXER = 16;

        public static final int LEFT_RAISE_MOTOR = 14;
        public static final int RIGHT_RAISE_MOTOR = 13;

        public static final double TARGET_Y = 1.974; // + Units.inchesToMeters(3);
        public static final double TARGET_X = 0.196;

        public static final double ELEVATOR_PIVOT_HEIGHT = Units.inchesToMeters(23.75);
        public static final double ARM_LENGTH = Units.inchesToMeters(10);
        public static final double FLYWHEELS_FROM_ARM = Units.inchesToMeters(7);

        public static final int ARM_MOTOR_LEFT = 60;
        public static final int ARM_MOTOR_RIGHT = 61;

        // relative to ground
        public static final double FLYWHEEL_OFFSET = Units.degreesToRadians(10); // 10 degrees tilted up
        public static final double ELEVATOR_X_OFFSET = Units.inchesToMeters(-1); // positive further back negative
                                                                                 // further
                                                                                 // forward

        public static final double FLYWHEEL_CIRCUMFERENCE = Math.PI * Units.inchesToMeters(4);

        public static final int INDEX_BEAMBREAKER = 0;

        public static final double INDEXER_FEED_SPEED = 0.85;

        public static final double AUTO_X_KP = 1.0;
        public static final double AUTO_X_KI = 0.0;
        public static final double AUTO_X_KD = 0.0;

        public static final double AUTO_Y_KP = 1.0;
        public static final double AUTO_Y_KI = 0.0;
        public static final double AUTO_Y_KD = 0.0;

        public static final double AUTO_THETA_KP = 1.0;
        public static final double AUTO_THETA_KI = 0.0;
        public static final double AUTO_THETA_KD = 0.0;

        public static final int FRONT_INTAKE_MOTOR = 15;

        public static final double ARM_ABSOLUTE_ENCODER_OFFSET = 0.6507399412684985;
        public static final double ARM_ABSOLUTE_CONVERSION_FACTOR = 49.87;

        public static String VISION_LIMELIGHT = "limelight-vanap";
        public static String DRIVER_LIMELIGHT = "limelight-panav";

        public static final boolean tuningMode = true;

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
