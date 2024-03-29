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
public class Drive extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain drivetrain;
  private Supplier<ChassisSpeeds> speedsSupplier;

  /**
   * Creates a new Drive.
   *
   * @param drivetrain The drivetrain subsystem
   * @param speedsSupplier The ChassisSpeeds supplier
   */
  public Drive(Drivetrain drivetrain, Supplier<ChassisSpeeds> speedsSupplier) {
    this.drivetrain = drivetrain;
    this.speedsSupplier = speedsSupplier;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.setSpeeds(speedsSupplier.get());
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
