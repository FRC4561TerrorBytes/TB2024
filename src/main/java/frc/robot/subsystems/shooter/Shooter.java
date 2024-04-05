// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.util.NoteVisualizer;

/** Add your docs here. */
public class Shooter extends SubsystemBase {

    private ShooterIO io;
    private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();

    private Indexer indexer;

    private final SysIdRoutine sysId;
    public double m_velocitySetpoint;
    private double m_angle;
    private double m_height;

    private double xOffset = 0.0;

    private double m_pivotAngle;

    public Shooter(ShooterIO io, Indexer indexer) {
        this.io = io;

        switch (Constants.currentMode) {
            case REAL:
            case REPLAY:

                break;
            case SIM:

                break;
            default:

                break;
        }
        SignalLogger.setPath("/media/sda1/");
        this.indexer = indexer;

        setAngleMap();

        // Configure SysId
        sysId =
          new SysIdRoutine(
            new SysIdRoutine.Config(
              null,
              null,
              null,
              (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
            SignalLogger.start();
    }

    private void setAngleMap() {
      angleMap.put(Units.inchesToMeters(31), -4.7);
      angleMap.put(Units.inchesToMeters(39), -5.0);
      angleMap.put(Units.inchesToMeters(47), -5.7);
      angleMap.put(Units.inchesToMeters(55), -6.2);
      angleMap.put(Units.inchesToMeters(62), -7.2);
      angleMap.put(Units.inchesToMeters(67), -7.3);
      angleMap.put(Units.inchesToMeters(68), -7.4);
      angleMap.put(Units.inchesToMeters(70), -7.75);
      angleMap.put(Units.inchesToMeters(71), -7.8);
      angleMap.put(Units.inchesToMeters(73), -7.9);
      angleMap.put(Units.inchesToMeters(78), -8.125);
      // angleMap.put(Units.inchesToMeters(79), -8.5);
      angleMap.put(Units.inchesToMeters(80), -8.25);
      angleMap.put(Units.inchesToMeters(88), -8.75);
      angleMap.put(Units.inchesToMeters(94), -9.0);
      angleMap.put(Units.inchesToMeters(98), -9.25);
      angleMap.put(Units.inchesToMeters(104), -9.35);
      angleMap.put(Units.inchesToMeters(110), -9.4);
      angleMap.put(Units.inchesToMeters(112), -9.45);
      angleMap.put(Units.inchesToMeters(114), -9.6);
      angleMap.put(Units.inchesToMeters(123), -9.8);
    }

    public double interpolateArmAngle(double distanceMeters) {
      return angleMap.get(distanceMeters);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter/IO", inputs);

        Logger.recordOutput("Shooter/Angle", Units.radiansToDegrees(m_angle));
        Logger.recordOutput("Shooter/Height", m_height);
    }

        /** Run open loop at the specified voltage. */
    public void runVolts(double volts) {
    io.setVoltage(volts);
    }

  public double findVelocity(double x){
    return Math.sqrt((((
                    (-1*(9.8*Math.pow(-x, 2)))
              /((Constants.TARGET_Y + m_height)/(-x*Math.tan(m_angle))))
                /Math.pow(Math.cos(m_angle), 2)))
                                /2);
  }

  // public double findTrajectoryPoint(double x, double distance){
  //   return((distance + x)*Math.tan(m_angle)-((9.8*Math.pow((distance + x), 2)) / (2*Math.pow(findVelocity(distance), 2) * Math.pow(Math.cos(m_angle), 2))));
  // }

  @AutoLogOutput(key = "Shooter/distance to tags")
  public double findFlatDistanceWithVision() {
    NetworkTable chair = NetworkTableInstance.getDefault().getTable("limelight-vanap");
    NetworkTableEntry ty = chair.getEntry("ty");
    double targetOffsetAngleVert = ty.getDouble(0.0);

    double llMountAngleDeg = 26.5;
    double llHeightIn = 19.75;
    double targetHeightIn = 59.5;
    double llToFrontRailIn = 4;

    double angleToGoalDeg = llMountAngleDeg + targetOffsetAngleVert;

    double distanceInches = (targetHeightIn - llHeightIn) / Math.tan(Units.degreesToRadians(angleToGoalDeg));

    return (distanceInches - llToFrontRailIn);
  }

  @AutoLogOutput(key = "Shooter/straight line angle")
  public double findStraightLineAngle(){
    NetworkTable chair = NetworkTableInstance.getDefault().getTable("limelight-vanap");
    NetworkTableEntry ty = chair.getEntry("ty");
    double targetOffsetAngleVert = ty.getDouble(0.0);

    double llMountAngleDeg = 25.0;

    double angleToGoalDeg = llMountAngleDeg + targetOffsetAngleVert;

    return angleToGoalDeg + 2;
  }

  public double calculateArmRotations(){
    double degrees = findStraightLineAngle() - Units.radiansToDegrees(Constants.FLYWHEEL_OFFSET);
    return degreesToArmRotations(degrees);
  }
  //use this to determine how many rotations 90 degrees is
  //because 0 is 90 and straight down is 0
  double straightDownRotations = -12.0;
  public double degreesToArmRotations(double degrees){
    //because 0 degrees is 90 relative to elevator
    degrees = degrees - 90;
    double encoderToArm = Math.abs(straightDownRotations)*4;
    return Units.degreesToRotations(degrees)*encoderToArm;
  }

  // public double calculateFlywheelSpeed(){
  //   m_angle = Units.degreesToRadians(findStraightLineAngle());

  //   double xOffset = (Constants.ARM_LENGTH*Math.sin(m_angle - Constants.FLYWHEEL_OFFSET)) + (Constants.FLYWHEELS_FROM_ARM*Math.sin(Units.degreesToRadians(90) - m_angle)) + Constants.ELEVATOR_X_OFFSET;

  //   //need this here because it is used in find velocity function
  //   m_height = Constants.ELEVATOR_PIVOT_HEIGHT-(Constants.ARM_LENGTH*Math.cos(m_angle - Constants.FLYWHEEL_OFFSET)) + (Constants.FLYWHEELS_FROM_ARM*Math.sin(m_angle));

  //   //distance from flywheel exit using flat distance to frame and subtracting(closer) the arm based off angle using trig
  //   m_velocitySetpoint = findVelocity(findFlatDistanceWithVision() + (Units.inchesToMeters(26.0)/2) - xOffset)*6;
  //   return m_velocitySetpoint;
  // }

  @AutoLogOutput(key = "Shooter/VelocitySetpoint")
  public double getVelocity(){
    return m_velocitySetpoint;
  }

  @AutoLogOutput(key = "Shooter/Pivot Angle")
  public double getPivotAngle(){
    return Units.radiansToDegrees(m_pivotAngle);
  }

  public void setFlywheelSpeed(double velocity){
    io.setFlywheelSpeed(velocity);
  }
  public void stopFlywheel(){
    io.stopFlywheel();
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public boolean flywheelUpToSpeed(double mps){
    return inputs.shooterVelocityMPS >= mps;
  }

  public boolean noteShot(){
    //return the beam break after the flywheels here
    return false;
  }

  public void idleFlywheels() {
    if (indexer.noteInIndexer()) {
      setFlywheelSpeed(10);
    } else {
      stopFlywheel();
    }
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Returns a command that launches a note. */
  public Command launchCommand() {
    return Commands.sequence(
                NoteVisualizer.shoot(m_velocitySetpoint, Units.radiansToDegrees(m_angle), m_height, xOffset),
            Commands.idle());
  }
}