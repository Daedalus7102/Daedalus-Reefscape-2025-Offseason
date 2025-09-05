package frc.robot.subsystems.drive;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class SwerveDrive extends SubsystemBase{

    // Swerve
    private SwerveModule m_swerveModule1 = SwerveConstants.kFrontLeft;
    private SwerveModule m_swerveModule2 = SwerveConstants.kFrontRight;
    private SwerveModule m_swerveModule3 = SwerveConstants.kBackLeft;
    private SwerveModule m_swerveModule4 = SwerveConstants.kBackRight;

    private final Pigeon2 m_gyro = new Pigeon2(13, "rio");
    private SwerveDriveOdometry m_odometry;
    private final SwerveDriveKinematics m_kinematics = SwerveConstants.kKinematics;
    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds();

    // Field
    private final Field2d m_field = new Field2d();
    
    // Singleton instance
    private static SwerveDrive m_instance = null;

    public static SwerveDrive getInstance() {
        if (m_instance == null) {
            m_instance = new SwerveDrive();
        }
        return m_instance;
    }

    // Control Inputs
    private DoubleSupplier m_translationX, m_translationY, m_rotationOmega;
    private DoubleSupplier m_padX, m_padY;
    private double m_deadband = 0.15d;
    private boolean m_fieldRelativeTeleop = true;

    private SwerveDrive() {
        m_gyro.reset();
        SmartDashboard.putData("Field", m_field);
        SmartDashboard.putBoolean("FieldRelativeTeleop", m_fieldRelativeTeleop);
        SmartDashboard.putNumber("DeadZone", m_deadband);
        SmartDashboard.putNumber("Gyro", m_gyro.getRotation2d().getDegrees());
        m_odometry = new SwerveDriveOdometry(
            m_kinematics, m_gyro.getRotation2d(),
            getSwerveModulePositions(), new Pose2d(0, 0, new Rotation2d())
        );
    }

    public void setJoystickSuppliers(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier omegaInput){
        m_translationX = xInput;
        m_translationY = yInput;
        m_rotationOmega = omegaInput;
    }

    public void setDPadSuppliers(DoubleSupplier xInput, DoubleSupplier yInput){
        m_padX = xInput;
        m_padY = yInput;
    }

    private boolean isJoystickInputPresent() {
        return (!Stream
            .of(m_translationX.getAsDouble(), m_translationY.getAsDouble(),
                m_rotationOmega.getAsDouble())
            .filter((input) -> MathUtil.applyDeadband(input, m_deadband) != 0)
            .collect(Collectors.toList()).isEmpty());
    }

    private boolean isDPadInputPresent() {
        return (!Stream
            .of(m_padX.getAsDouble(), m_padY.getAsDouble())
            .filter((input) -> input != 0)
            .collect(Collectors.toList()).isEmpty());
    }

    @Override
    public void periodic(){
        // Rotation2d angle = new Rotation2d(Math.atan2(m_translationY.getAsDouble(), m_translationX.getAsDouble()));
        // double speed = Math.hypot(m_translationX.getAsDouble(), m_translationY.getAsDouble());
        // SwerveModuleState state = new SwerveModuleState(speed * SwerveConstants.kDriveMaxSpeed, angle);

        // SmartDashboard.putNumber("Joystick Angle", angle.getDegrees());
        // SmartDashboard.putNumber("Module1 Angle", m_swerveModule1.getModuleRotation());
        // SmartDashboard.putNumber("Module2 Angle", m_swerveModule2.getModuleRotation());
        // SmartDashboard.putNumber("Module3 Angle", m_swerveModule3.getModuleRotation());
        // SmartDashboard.putNumber("Module4 Angle", m_swerveModule4.getModuleRotation());

        // m_swerveModule4.setDesiredState(state);
        // m_testModule2.setDesiredState(state);
        // m_testModule3.setDesiredState(state);
        // m_testModule4.setDesiredState(state);

        boolean FRT = SmartDashboard.getBoolean("FieldRelativeTeleop", true);
        if(FRT != m_fieldRelativeTeleop){ m_fieldRelativeTeleop = FRT;}
        double DZ = SmartDashboard.getNumber("DeadBand_Drift", 0);
        if(DZ != m_deadband){ m_deadband = DZ;}
        SmartDashboard.putNumber("Gyro", m_gyro.getRotation2d().getDegrees());
        SmartDashboard.putBoolean("DriverJoystick", isJoystickInputPresent());

        teleopDrive().schedule();
        setSwerveModuleStates(m_chassisSpeeds);
        m_odometry.update(m_gyro.getRotation2d(), getSwerveModulePositions());
        m_field.setRobotPose(m_odometry.getPoseMeters());

    }

    private SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] positions = {
            m_swerveModule1.getModulePosition(),
            m_swerveModule2.getModulePosition(),
            m_swerveModule3.getModulePosition(),
            m_swerveModule4.getModulePosition()
        };
        return positions;
    }

    private void setSwerveModuleStates(ChassisSpeeds chassisSpeeds){
        SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.kDriveMaxSpeed);
        m_swerveModule1.setDesiredState(swerveModuleStates[0]);
        m_swerveModule2.setDesiredState(swerveModuleStates[1]);
        m_swerveModule3.setDesiredState(swerveModuleStates[2]);
        m_swerveModule4.setDesiredState(swerveModuleStates[3]);
    }

    private Command teleopDrive() {
        return new RunCommand(() -> {
            double X = MathUtil.applyDeadband(m_translationX.getAsDouble(), m_deadband)*SwerveConstants.kDriveMaxSpeed;
            double Y = MathUtil.applyDeadband(m_translationY.getAsDouble(), m_deadband)*SwerveConstants.kDriveMaxSpeed;
            double Omega = MathUtil.applyDeadband(m_rotationOmega.getAsDouble(), m_deadband)*SwerveConstants.kTurnMaxSpeed;

            drive(X, Y, Omega, m_fieldRelativeTeleop, 0.02);
        }, this);
    }

    private void drive(double x, double y, double omega, boolean fieldRelative, double periodSenconds){
        m_chassisSpeeds = ChassisSpeeds.discretize(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(x, y, omega, m_gyro.getRotation2d())
                : new ChassisSpeeds(x, y, omega), periodSenconds);
    }

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
