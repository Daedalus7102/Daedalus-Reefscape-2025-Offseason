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
  public static final CommandPS5Controller m_driverController = new CommandPS5Controller(0);
  // public static final CommandPS5Controller m_operatorController = new CommandPS5Controller(1);

  // Subsystems
  private SwerveDrive m_swerveSubsystem = SwerveDrive.getInstance();

  // Autonomous
  private SendableChooser<Command> m_autoChooser;

  public RobotContainer() {
    // m_autoChooser = AutoBuilder.buildAutoChooser();
    // SmartDashboard.putData("Auto Mode", m_autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    m_swerveSubsystem.setJoystickSuppliers(
      () -> -m_driverController.getLeftY(), 
      () -> -m_driverController.getLeftX(), 
      () -> -m_driverController.getRightX()
    );
    // swerveSubsystem.setDefaultCommand(
    // swerveSubsystem.drive(
    // () -> driverController.getLeftY(),
    // () -> driverController.getLeftX(),
    // () -> driverController.getRightX(),
    // () -> driverController.getR2() > 0.5,
    // () -> driverController.getL2() > 0.5
    // )
    // );
  }

  public Command getAutonomousCommand() {
    return new Command() {
      
    };
    // return m_autoChooser.getSelected();
  }
}
