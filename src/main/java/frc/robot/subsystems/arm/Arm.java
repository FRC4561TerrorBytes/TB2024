// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Add your docs here. */
public class Arm extends SubsystemBase {
    private ArmIO io;
    private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    
    public MechanismLigament2d arm;
    public MechanismLigament2d shooter;

    Mechanism2d mech = new Mechanism2d(4, 4);

    MechanismRoot2d root = mech.getRoot("base", 2, 0);

    public Arm(ArmIO io) {
        this.io = io;

        arm = root.append(new MechanismLigament2d("arm", 0.324, 0, 6, new Color8Bit(Color.kPurple)));
        shooter = arm.append(new MechanismLigament2d("shooter", 0.118, 120, 4, new Color8Bit(Color.kHotPink)));
        Logger.recordOutput("Mech 2d", mech);
    }

    public void runArmWithVoltage(double volts) {
        io.setArmVoltage(volts);
    }

    public void setArmSetpoint(double setpoint) {
        io.setArmSetpoint(setpoint);
    }

    public boolean armAtSetpoint(){
        return io.armAtSetpoint();
    }
    
    public double getArmAngleDegrees() {
        return (inputs.armRelativeAngleRotations + 12.25) / Constants.ARM_ABSOLUTE_CONVERSION_FACTOR * 360.0;
    }

    public void incrementArmAngle(double inc) {
        io.incrementArmAngle(inc);
    }
    
    public void decrementArmAngle(double inc) {
        io.decrementArmAngle(inc);
    }

    public void seedEncoders() {
        io.seedEncoders();
    }

    public void stopArm() {
        io.setArmVoltage(0);
    }

    public void nudge(double degrees){
        System.out.println("\n\n\n\n\n\n\n\n\nArm nudge is running");
        io.nudge(degrees);
    }

    public double getArmEncoderRotation(){
        return io.getArmEncoderRotation();
    }

    public double getAbsoluteRotations(){
        return io.getAbsoluteRotations();
    }

    @AutoLogOutput(key = "Arm/CAN Disconnect")
    public boolean getDisconnect() {
        return io.getDisconnect();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm/IO", inputs);

        arm.setAngle(getArmAngleDegrees());

        Pose3d actualArmPose = new Pose3d(-0.025, 0, 0.605, new Rotation3d(Units.degreesToRadians(getArmAngleDegrees()), 0, Units.degreesToRadians(90)));
        Pose3d requestedArmPose = new Pose3d(-0.025, 0, 0.605, new Rotation3d(Units.degreesToRadians(((inputs.armSetpoint + 12.25) / Constants.ARM_ABSOLUTE_CONVERSION_FACTOR * 360.0)), 0, Units.degreesToRadians(90)));
    
        Logger.recordOutput("Poses/Actual Arm", actualArmPose);
        Logger.recordOutput("Poses/Requested Arn", requestedArmPose);
    }
}
