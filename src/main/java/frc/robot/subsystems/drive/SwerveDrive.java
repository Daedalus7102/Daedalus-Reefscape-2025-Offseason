package frc.robot.subsystems.drive;

public class SwerveDrive {
    /*
    =====================================================
    GUÍA DE USO DEL SUBSISTEMA SWERVE (DriveTrain.java)
    =====================================================

    Este archivo implementa el subsistema de drivetrain
    con módulos swerve. Aquí tienes una guía comentada
    sobre los métodos más importantes para controlarlo
    en teleoperado y autónomo.

    -----------------------------------------------------
    INICIALIZACIÓN
    -----------------------------------------------------
    DriveTrain dt = DriveTrain.getInstance(); // Singleton

    // Configuración de entradas (se hace en robotInit o teleopInit)
    dt.setJoystickSuppliers(xSup, ySup, omegaSup);   // Joystick: X/Y traslación, Omega rotación
    dt.setDPadSuppliers(dpadX, dpadY);              // D-Pad: pequeños movimientos discretos
    dt.setElevatorHeightSupplier(() -> elevatorHeightActual);
    // Ajusta la velocidad si el elevador está alto (seguridad)

    -----------------------------------------------------
    MODOS DE MANEJO (STATE MACHINE)
    -----------------------------------------------------
    // Cambiar el estado actual de conducción (teleop/idle/etc.)
    dt.setState(DriveTrain.DriveTrainState.JOYSTICKS).schedule();  // Teleop con joystick
    dt.setState(DriveTrain.DriveTrainState.D_PAD).schedule();      // Conducción con D-Pad
    dt.setState(DriveTrain.DriveTrainState.IDLE).schedule();       // Stop/Idle

    // Obtener el estado actual
    DriveTrain.DriveTrainState estado = dt.getState();

    -----------------------------------------------------
    CONDUCCIÓN DIRECTA (Robot-Relative)
    -----------------------------------------------------
    // Conducción manual sin pasar por máquina de estados.
    // Útil para auton o rutinas programadas.
    dt.driveRobotRelative(new ChassisSpeeds(vx, vy, omega));

    -----------------------------------------------------
    ODOMETRÍA Y POSE
    -----------------------------------------------------
    // Pose actual del robot en el campo
    Pose2d pose = dt.getPose();

    // Resetear pose (ej. al inicio de auton)
    dt.resetPose(new Pose2d(0, 0, new Rotation2d()));

    // Obtener velocidades medidas
    ChassisSpeeds speeds = dt.getSpeeds();

    -----------------------------------------------------
    AUTONOMÍA (PathPlanner / AutoBuilder)
    -----------------------------------------------------
    // AutoBuilder ya está configurado en el constructor
    Command auto = AutoBuilder.buildAuto("NombreDelAuto");
    // -> ejecuta trayectorias cargadas desde PathPlanner
    */
}
