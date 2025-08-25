// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

import frc.robot.subsystems.drive.SwerveDrive;

public class RobotContainer {

  // Controllers
  public static final CommandPS5Controller driverController = new CommandPS5Controller(0);
  public static final CommandPS5Controller operatorController = new CommandPS5Controller(1);

  // Subsystems
  private SwerveDrive swerveSubsystem = new SwerveDrive();

  // Autonomous
  private SendableChooser<Command> autoChooser;

  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    // swerveSubsystem.setDefaultCommand(
    //   swerveSubsystem.drive(
    //     () -> driverController.getLeftY(),
    //     () -> driverController.getLeftX(),
    //     () -> driverController.getRightX(),
    //     () -> driverController.getR2() > 0.5,
    //     () -> driverController.getL2() > 0.5
    //   )
    // );
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
