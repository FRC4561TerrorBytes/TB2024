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
import frc.robot.RobotContainer.shootPositions;
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

    private double armOffset = 0.35;

    private void setAngleMap() {
      angleMap.put(Units.inchesToMeters(41), -3.7);
      angleMap.put(Units.inchesToMeters(57), -5.1);
      angleMap.put(Units.inchesToMeters(70), -5.2);
      angleMap.put(Units.inchesToMeters(83), -6.3);
      angleMap.put(Units.inchesToMeters(94), -7.0);
      angleMap.put(Units.inchesToMeters(107), -7.4);
      angleMap.put(Units.inchesToMeters(119), -7.75);
      angleMap.put(Units.inchesToMeters(139), -8.0);
      angleMap.put(Units.inchesToMeters(155), -8.4);
      angleMap.put(Units.inchesToMeters(170), -8.5);
      angleMap.put(Units.inchesToMeters(186), -8.8);
      angleMap.put(Units.inchesToMeters(200), -9.0);
      angleMap.put(Units.inchesToMeters(222), -9.1);
      angleMap.put(Units.inchesToMeters(237), -9.2);
      angleMap.put(Units.inchesToMeters(247), -9.3);
      angleMap.put(Units.inchesToMeters(285), -9.5);
    }

    public double interpolateArmAngle(double distanceMeters) {
      return angleMap.get(distanceMeters) + armOffset;
    }

    public double regressionArmAngle(double distanceMeters) {
      return -3.0225 - 3.9907 * Math.log(distanceMeters);
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

    double llMountAngleDeg = 27.5;
    double llHeightIn = 18;
    double targetHeightIn = 59.5;
    double llToFrontRailIn = 13;

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
  
  @AutoLogOutput(key = "Shooter/CAN Disconnect")
  public boolean getDisconnect(){
    return io.getDisconnect();
  }

  public boolean flywheelUpToSpeed(double mps){
    return inputs.shooterVelocityMPS >= (mps * 0.95);
  }

  public boolean noteShot(){
    //return the beam break after the flywheels here
    return false;
  }

  public void idleFlywheels(shootPositions armPosition) {
    if (indexer.noteInIndexer() && !armPosition.equals(shootPositions.AMP)) {
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