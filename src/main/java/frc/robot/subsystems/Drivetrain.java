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
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Drivetrain subsystem controlled with ChassisSpeeds
 */
public class Drivetrain extends SubsystemBase {
  TalonSRX frontLeft = new TalonSRX(DrivetrainConstants.FrontLeft);
  TalonSRX frontRight = new TalonSRX(DrivetrainConstants.FrontRight);
  TalonSRX backLeft = new TalonSRX(DrivetrainConstants.BackLeft);
  TalonSRX backRight = new TalonSRX(DrivetrainConstants.BackRight);

  public Drivetrain() {
    frontLeft.setInverted(false);
    frontRight.setInverted(true);
    backLeft.setInverted(false);
    backRight.setInverted(true);
  }

  @Override
  public void periodic() {

  }

  public void setSpeeds(double xSpeed, double ySpeed, double rotate) {
    double frontLeftSpeed = xSpeed + ySpeed + rotate;
    double frontRightSpeed = -xSpeed + ySpeed - rotate;
    double backLeftSpeed = -xSpeed + ySpeed + rotate;
    double backRightSpeed = xSpeed + ySpeed - rotate;

    SmartDashboard.putNumber("Front Left Setpoint", frontLeftSpeed);
    SmartDashboard.putNumber("Front Right Setpoint", frontRightSpeed);
    SmartDashboard.putNumber("Back Left Setpoint", backLeftSpeed);
    SmartDashboard.putNumber("Back Right Setpoint", backRightSpeed);

    frontLeft.set(ControlMode.PercentOutput, frontLeftSpeed);
    frontRight.set(ControlMode.PercentOutput, frontRightSpeed);
    backLeft.set(ControlMode.PercentOutput, backLeftSpeed);
    backRight.set(ControlMode.PercentOutput, backRightSpeed);
  }
}
