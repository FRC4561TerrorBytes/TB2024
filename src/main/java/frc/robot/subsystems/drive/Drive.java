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

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import java.util.Arrays;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.SwerveSetpoint;


public class Drive extends SubsystemBase {
  private static final double MAX_LINEAR_SPEED = Units.feetToMeters(14.5);
  private static final double TRACK_WIDTH_X = Units.inchesToMeters(26.0);
  private static final double TRACK_WIDTH_Y = Units.inchesToMeters(26.0);
  private static final double DRIVE_BASE_RADIUS =
      Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
  private static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;
  private static final Translation2d speakerPose = new Translation2d(0.0, 5.55);
  private static final Translation2d blueSpeaker = new Translation2d(0.225, 5.55);

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  private final VisionIO visionIO;
  private final VisionIOInputsAutoLogged visionInputs = new VisionIOInputsAutoLogged();

  private BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();

  private final SysIdRoutine sysId;

  private final Module[] modules = new Module[4]; // FL, FR, BL, BR

  private boolean modulesOrienting = false;
  private SwerveSetpoint currentSetpoint =
  new SwerveSetpoint(
      new ChassisSpeeds(),
      new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
      });

  private double prevXAccel = 0.0;
  private double prevYAccel = 0.0;
  private int collisions = 0;

  // private final Orchestra m_orchestra = new Orchestra("verySecretMusicFile.chrp"); ///home/lvuser/deploy/verySecretMusicFile.chrp

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  private SwerveDrivePoseEstimator m_poseEstimator =
    new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());


  //CHANGE THE NUMBERS IN THE VECTOR BUILDER
  // private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

  public Drive(
      GyroIO gyroIO,
      VisionIO visionIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {

    this.gyroIO = gyroIO;
    this.visionIO = visionIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Take tags that are out of tolerance out of this list
    int[] validIds = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    LimelightHelpers.SetFiducialIDFiltersOverride(Constants.VISION_LIMELIGHT, validIds);

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPose,
        () -> kinematics.toChassisSpeeds(getModuleStates()),
        this::runVelocity,
        new HolonomicPathFollowerConfig(
            new PIDConstants(1.86, 0, 0), 
            new PIDConstants(1.2,0,0.07),
            MAX_LINEAR_SPEED, DRIVE_BASE_RADIUS, new ReplanningConfig()),
        () -> 
        DriverStation.getAlliance().isPresent()
          && DriverStation.getAlliance().get() == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    sysId = 
        new SysIdRoutine(
          new SysIdRoutine.Config(
            null,
            null,
            null,
            (state) -> Logger.recordOutput("Drive/SysIDState", state.toString())),
          new SysIdRoutine.Mechanism(
            (voltage) -> {
              for (int i = 0; i < 4; i++) {
                modules[i].runCharacterization(voltage.in(Volts));
              }
            },
            null,
            this));

        //m_orchestra.addInstrument(modules[1].getDriveTalon());

    int[] validIDs = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    LimelightHelpers.SetFiducialIDFiltersOverride(Constants.VISION_LIMELIGHT, validIDs);
  }

  public void periodic() {
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);

    visionIO.updateInputs(visionInputs);
    Logger.processInputs("Vision/Limelight", visionInputs);

    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }
    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Read wheel positions and deltas from each module
    SwerveModulePosition[] modulePositions = getModulePositions();
    SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
      moduleDeltas[moduleIndex] =
          new SwerveModulePosition(
              modulePositions[moduleIndex].distanceMeters
                  - lastModulePositions[moduleIndex].distanceMeters,
              modulePositions[moduleIndex].angle);
      lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
    }

    // Update gyro angle
    if (gyroInputs.connected) {
      // Use the real gyro angle
      rawGyroRotation = gyroInputs.yawPosition;
    } else {
      // Use the angle delta from the kinematics and module deltas
      Twist2d twist = kinematics.toTwist2d(moduleDeltas);
      rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
    }

    // Get current acceleration values
    double XAccel = accelerometer.getX();
    double YAccel = accelerometer.getY();

    // Calculate jerk by subtracting current accel with previous and dividing by loop time
    double xJerk = (XAccel - prevXAccel) / 0.2;
    double yJerk = (YAccel - prevYAccel) / 0.2;

    // Set Previous accel for next loop
    prevXAccel = XAccel;
    prevYAccel = YAccel;

    // Discard data if collision is detected
    if (xJerk > -8.5 && yJerk > -8.5) {
      m_poseEstimator.update(rawGyroRotation, modulePositions);
    } else {
      collisions++;
    }

    Logger.recordOutput("Vision/Collisions Detected", collisions);

    Logger.recordOutput("Vision/xJerk", xJerk);
    Logger.recordOutput("Vision/yJerk", yJerk);
    
    LimelightHelpers.SetRobotOrientation(Constants.VISION_LIMELIGHT, getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);

    if(Math.abs(Units.radiansToDegrees(gyroInputs.yawVelocityRadPerSec)) < 720 && visionInputs.mt2TagCount > 0){
      m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, Units.degreesToRadians(5)));
      m_poseEstimator.addVisionMeasurement(visionInputs.mt2Pose, visionInputs.mt2Timestamp);
    }

    // I wonder if we had a command factory for a note align inside of drive bc of IO later stuf???
  }

  public void getSpeakerDistance()
  {
    Translation2d current = new Translation2d(getPose().getX(), getPose().getY());
    if(AllianceFlipUtil.shouldFlip())
    {
      Translation2d redSpeaker = new Translation2d(AllianceFlipUtil.apply(speakerPose.getX()), AllianceFlipUtil.apply(speakerPose.getY()));
      double dist = Math.sqrt(
        Math.pow(
          Math.abs(current.getX()-redSpeaker.getX()), 2) + Math.pow(Math.abs(current.getY()-redSpeaker.getY()), 2));
    } else
    {
      double dist = Math.sqrt(
        Math.pow(
          Math.abs(current.getX()-blueSpeaker.getX()), 2) + Math.pow(Math.abs(current.getY()-blueSpeaker.getY()), 2));
    }
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(volts);
    }
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (var module : modules) {
      driveVelocityAverage += module.getCharacterizationVelocity();
    }
    return driveVelocityAverage / 4.0;
  }

  public double[] getWheelRadiusCharacterizationPosition() {
    return Arrays.stream(modules).mapToDouble(Module::getPositionRads).toArray();
  }

  public void runWheelRadiusCharacterization(double omegaSpeed) {
    runVelocity(new ChassisSpeeds(0.0, 0.0, omegaSpeed));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    //return pose;
    return m_poseEstimator.getEstimatedPosition();
  }

  @AutoLogOutput(key = "Odometry/Robot 3d")
  public Pose3d get3dPose() {
    return new Pose3d(m_poseEstimator.getEstimatedPosition());
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }


  public Rotation2d getRotation() {
    //return pose.getRotation();
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    m_poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

    /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    m_poseEstimator.addVisionMeasurement(visionPose, timestamp);
  }

  public void resetGyro() {
    gyroIO.resetGyro();
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return MAX_LINEAR_SPEED;
  }

  /** Returns the maximum angular speed in radians per sec. */
  @AutoLogOutput(key = "Drive/Max Angular Speed Rad per s")
  public double getMaxAngularSpeedRadPerSec() {
    return MAX_ANGULAR_SPEED;
  }

  private LimelightTarget_Fiducial getClosestTag(String cameraName) {
    double closest = 100;
    LimelightTarget_Fiducial target = null;
    LimelightTarget_Fiducial[] targetList = LimelightHelpers.getLatestResults(cameraName).targetingResults.targets_Fiducials;
    for (LimelightTarget_Fiducial i : targetList) {
      double value = i.tx;
      if (value < closest) {
        closest = value;
        target = i;
      }
    }
    return target;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
    };
  }

  public Pose2d getSpeakerPose(){
    Pose2d speaker = new Pose2d();
    if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue){
      speaker = new Pose2d(new Translation2d(0, 5.54), new Rotation2d());
    }
    else if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
      speaker = new Pose2d(new Translation2d(16.54, 5.54), new Rotation2d());
    }
    return speaker;
  }

  public double getDistanceFromSpeaker(){
    return getSpeakerPose().getTranslation().getDistance(getPose().getTranslation());
  }

  public double getRotationFromSpeaker(){
    
    //using the convention of 0 facing forward on blue origin(facing red) and 0 facing forward on red is facing blue wall
    Pose2d relative = getSpeakerPose().relativeTo(getPose());
    double angle = Units.radiansToDegrees(Math.atan(relative.getY()/relative.getX()));
    return angle;
  }

    /**
   * Returns command that orients all modules to {@code orientation}, ending when the modules have
   * rotated.
   */
  public Command orientModules(Rotation2d orientation) {
    return orientModules(new Rotation2d[] {orientation, orientation, orientation, orientation});
  }

  /**
   * Returns command that orients all modules to {@code orientations[]}, ending when the modules
   * have rotated.
   */
  public Command orientModules(Rotation2d[] orientations) {
    return run(() -> {
          SwerveModuleState[] states = new SwerveModuleState[4];
          for (int i = 0; i < orientations.length; i++) {
            modules[i].runSetpoint(
                new SwerveModuleState(0.0, orientations[i]));
            states[i] = new SwerveModuleState(0.0, modules[i].getAngle());
          }
          currentSetpoint = new SwerveSetpoint(new ChassisSpeeds(), states);
        })
        .until(
            () ->
                Arrays.stream(modules)
                    .allMatch(
                        module ->
                            EqualsUtil.epsilonEquals(
                                module.getAngle().getDegrees(),
                                module.getState().angle.getDegrees(),
                                2.0)))
        .beforeStarting(() -> modulesOrienting = true)
        .finallyDo(() -> modulesOrienting = false)
        .withName("Orient Modules");
  }

  public static Rotation2d[] getCircleOrientations() {
    return Arrays.stream(Constants.MODULE_TRANSLATIONS)
        .map(translation -> translation.getAngle().plus(new Rotation2d(Math.PI / 2.0)))
        .toArray(Rotation2d[]::new);
  }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
      return sysId.quasistatic(direction);
    }
  
    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
      return sysId.dynamic(direction);
    }
}