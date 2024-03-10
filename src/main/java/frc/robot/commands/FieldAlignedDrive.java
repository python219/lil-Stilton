// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

/** 
 * Drive command that stores a speeds supplier for {@link Drivetrain} 
 */
public class FieldAlignedDrive extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain drivetrain;
  private Supplier<Double> xSpeedSupplier;
  private Supplier<Double> ySpeedSupplier;
  private Supplier<Double> rotateSpeedSupplier;
  private Supplier<Double> speedSupplier;
  private AHRS ahrs;

  /**
   * Creates a new Drive.
   *
   * @param drivetrain The drivetrain subsystem
   * @param speedsSupplier The ChassisSpeeds supplier
   */
  public FieldAlignedDrive(Drivetrain drivetrain, Supplier<Double> xSpeedSupplier, Supplier<Double> ySpeedSupplier, Supplier<Double> rotateSpeedSupplier, Supplier<Double> speedSupplier, AHRS ahrs) {
    this.drivetrain = drivetrain;
    this.xSpeedSupplier = xSpeedSupplier;
    this.ySpeedSupplier = ySpeedSupplier;
    this.rotateSpeedSupplier = rotateSpeedSupplier;
    this.speedSupplier = speedSupplier;
    this.ahrs = ahrs;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed;
    double ySpeed;
    double rotateSpeed;

    double magnitude = Math.sqrt(Math.abs(xSpeedSupplier.get()) + Math.abs(ySpeedSupplier.get()));
    double angle = Math.atan2(ySpeedSupplier.get() / magnitude, xSpeedSupplier.get() / magnitude);
    double newAngle = angle - Units.degreesToRadians(ahrs.getYaw());

    SmartDashboard.putNumber("Test Number", newAngle);

    if (!Double.isNaN(newAngle)) {
      xSpeed = Math.cos(newAngle) * speedSupplier.get();
      ySpeed = -Math.sin(newAngle) * speedSupplier.get();
    } else {
      xSpeed = 0;
      ySpeed = 0;
    }
    rotateSpeed = rotateSpeedSupplier.get() * speedSupplier.get();

    drivetrain.setSpeeds(xSpeed, ySpeed, rotateSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
