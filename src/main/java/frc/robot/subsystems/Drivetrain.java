// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.DrivetrainConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  TalonSRX frontLeft = new TalonSRX(DrivetrainConstants.FrontLeft);
  TalonSRX frontRight = new TalonSRX(DrivetrainConstants.FrontRight);
  TalonSRX backLeft = new TalonSRX(DrivetrainConstants.BackLeft);
  TalonSRX backRight = new TalonSRX(DrivetrainConstants.BackRight);

  MecanumDriveKinematics mecanumKinematics = new MecanumDriveKinematics(
    new Translation2d(-DrivetrainConstants.WheelXDistance, DrivetrainConstants.WheelYDistance),
    new Translation2d(DrivetrainConstants.WheelXDistance, DrivetrainConstants.WheelYDistance),
    new Translation2d(-DrivetrainConstants.WheelXDistance, -DrivetrainConstants.WheelYDistance),
    new Translation2d(DrivetrainConstants.WheelXDistance, -DrivetrainConstants.WheelYDistance)
  );

  /** Creates a new ExampleSubsystem. */
  public Drivetrain() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void setSpeeds(ChassisSpeeds speeds) {
    MecanumDriveWheelSpeeds wheelSpeeds = mecanumKinematics.toWheelSpeeds(speeds);
    frontLeft.set(ControlMode.Velocity, wheelSpeeds.frontLeftMetersPerSecond / DrivetrainConstants.VelocityConversionFactor);
    frontRight.set(ControlMode.Velocity, wheelSpeeds.frontRightMetersPerSecond / DrivetrainConstants.VelocityConversionFactor);
    backLeft.set(ControlMode.Velocity, wheelSpeeds.rearLeftMetersPerSecond / DrivetrainConstants.VelocityConversionFactor);
    backRight.set(ControlMode.Velocity, wheelSpeeds.rearRightMetersPerSecond / DrivetrainConstants.VelocityConversionFactor);
  }
}
