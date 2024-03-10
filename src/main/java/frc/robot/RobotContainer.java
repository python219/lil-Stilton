// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.FieldAlignedDrive;
import frc.robot.commands.RobotAlignedDrive;
import frc.robot.subsystems.Drivetrain;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandJoystick driverController =
      new CommandJoystick(OperatorConstants.DriverControllerPort);
  
  private final SendableChooser<Double> speedChooser = new SendableChooser<>();

  private AHRS ahrs = new AHRS(SPI.Port.kMXP);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    speedChooser.setDefaultOption("Normal Speed", DrivetrainConstants.DrivetrainNormalSpeed);
    speedChooser.addOption("Slow Speed", DrivetrainConstants.DrivetrainSlowSpeed);
    speedChooser.addOption("Fast Speed", DrivetrainConstants.DrivetrainFastSpeed);
    SmartDashboard.putData(speedChooser);

    ahrs.reset();
    SmartDashboard.putData(ahrs);
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    drivetrain.setDefaultCommand(
      new FieldAlignedDrive(
        drivetrain,
        () -> Math.pow(driverController.getRawAxis(0), 3),
        () -> Math.pow(driverController.getRawAxis(1), 3),
        () -> Math.pow(driverController.getRawAxis(2), 3),
        () -> speedChooser.getSelected(),
        ahrs
      )
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(drivetrain);
  }

  public void teleopInit() {
    
  }
}
