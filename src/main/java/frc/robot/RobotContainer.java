// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake;
import frc.robot.commands.AutoTake; 

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.vision.Limelight;

public class RobotContainer {

	// Controllers
	public static final CommandXboxController m_driverController = new CommandXboxController(0);
	public static final CommandPS5Controller m_operatorController = new CommandPS5Controller(1); // TODO: Change to Xbox if needed
	private final Limelight m_ll = new Limelight("limelight"); // ll = Limelight
	// public static final CommandXboxController m_operatorController = new CommandXboxController(1);

	// Subsystems
	private SwerveDrive m_swerveSubsystem = SwerveDrive.getInstance();
	private Elevator m_elevatorSubsystem = Elevator.getInstance();
	private final Intake m_intakeSubsystem = new Intake();

	// Autonomous
	private SendableChooser<Command> m_autoChooser;

	public RobotContainer() {
		configureBindings();
		m_autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto Mode", m_autoChooser);
	}

	private void configureBindings() {
		// Driver Controller
		m_swerveSubsystem.setJoystickSuppliers(
			() -> -m_driverController.getHID().getLeftY(),
			() -> -m_driverController.getHID().getLeftX(),
			() -> -m_driverController.getHID().getRightX()
		);
		m_swerveSubsystem.setDPadSuppliers(
			() -> {
				int pov = m_driverController.getHID().getPOV();
				if (pov == -1) return 0.0;
				return Math.round(Math.cos(Math.toRadians(pov)));
			},
			() -> {
				int pov = m_driverController.getHID().getPOV();
				if (pov == -1) return 0.0;
				return -Math.round(Math.sin(Math.toRadians(pov)));
			}
		);

		// Driver: Hold A if xbox to auto-approach coral using Limelight
		m_driverController.a().whileTrue(new AutoTake(m_swerveSubsystem, m_intakeSubsystem, m_ll));


		// -----------------------------
		// Operator: Intake bindings
		// -----------------------------

		// Roller: R1 = intake in, L1 = eject out
		m_operatorController.R1()
			.onTrue(m_intakeSubsystem.rollerIn())
			.onFalse(m_intakeSubsystem.rollerStop());

		m_operatorController.L1()
			.onTrue(m_intakeSubsystem.rollerOut())
			.onFalse(m_intakeSubsystem.rollerStop());

		// Pivot manual: POV up/down (hold = jog)
		m_operatorController.povUp()
			.onTrue(m_intakeSubsystem.pivotManual(+0.4))
			.onFalse(m_intakeSubsystem.stop());

		m_operatorController.povDown()
			.onTrue(m_intakeSubsystem.pivotManual(-0.4))
			.onFalse(m_intakeSubsystem.stop());

		// Setpoints (tap once)
		// TODO: check triangle and cross not alredy in use
		m_operatorController.triangle().onTrue(m_intakeSubsystem.pivotToStow());
		m_operatorController.cross().onTrue(m_intakeSubsystem.pivotToFloor());


		// Operator Controller
		// m_operatorController.cross()
		// 	.toggleOnTrue(m_elevatorSubsystem.moveElevator(0.2))
		// 	.toggleOnFalse(m_elevatorSubsystem.stopElevator());
		// m_operatorController.circle()
		// 	.toggleOnTrue(m_elevatorSubsystem.moveElevator(-0.2))
		// 	.toggleOnFalse(m_elevatorSubsystem.stopElevator());

		

	}

	public Command getAutonomousCommand() {
		return m_autoChooser.getSelected();
	}
}
