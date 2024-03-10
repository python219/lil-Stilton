// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

/** 
 * Drive command that stores a speeds supplier for {@link Drivetrain} 
 */
public class RobotAlignedDrive extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain drivetrain;
  private Supplier<Double> xSpeedSupplier;
  private Supplier<Double> ySpeedSupplier;
  private Supplier<Double> rotateSpeedSupplier;
  private Supplier<Double> speedSupplier;

  /**
   * Creates a new Drive.
   *
   * @param drivetrain The drivetrain subsystem
   * @param speedsSupplier The ChassisSpeeds supplier
   */
  public RobotAlignedDrive(Drivetrain drivetrain, Supplier<Double> xSpeedSupplier, Supplier<Double> ySpeedSupplier, Supplier<Double> rotateSpeedSupplier, Supplier<Double> speedSupplier) {
    this.drivetrain = drivetrain;
    this.xSpeedSupplier = xSpeedSupplier;
    this.ySpeedSupplier = ySpeedSupplier;
    this.rotateSpeedSupplier = rotateSpeedSupplier;
    this.speedSupplier = speedSupplier;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.setSpeeds(xSpeedSupplier.get() * speedSupplier.get(), ySpeedSupplier.get() * speedSupplier.get(), rotateSpeedSupplier.get() * speedSupplier.get());
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
