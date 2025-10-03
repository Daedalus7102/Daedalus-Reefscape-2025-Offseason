package frc.robot.subsystems.drive;

import java.util.function.DoubleSupplier;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.SwerveConstants;

public class SwerveDrive extends SubsystemBase {
    // Singleton instance
    private static volatile SwerveDrive m_instance = null;

    public static SwerveDrive getInstance() {
        if (m_instance == null) {
            synchronized (SwerveDrive.class) {
                if (m_instance == null) {
                    m_instance = new SwerveDrive();
                }
            }
        }
        return m_instance;
    }

    // Drive States
    public enum SwerveDriveState {
        JOYSTICKS,
        D_PAD,
        IDLE,
        LOCKED,
        ON_THE_FLY,
        AUTO,
    }

    private SwerveDriveState m_state = SwerveDriveState.IDLE;
    private SwerveDriveState m_lastState = null;

    // Swerve
    private final SwerveModule m_swerveModule1 = SwerveConstants.kFrontLeft;
    private final SwerveModule m_swerveModule2 = SwerveConstants.kFrontRight;
    private final SwerveModule m_swerveModule3 = SwerveConstants.kBackLeft;
    private final SwerveModule m_swerveModule4 = SwerveConstants.kBackRight;

    private final Pigeon2 m_gyro = new Pigeon2(SwerveConstants.kPigeonID, "rio");
    private SwerveDriveOdometry m_odometry;
    private final SwerveDriveKinematics m_kinematics = SwerveConstants.kKinematics;
    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds();

    // Field
    private final Field2d m_field = new Field2d();

    // Control Inputs
    private DoubleSupplier m_translationX = () -> 0.0, m_translationY = () -> 0.0, m_rotationOmega = () -> 0.0;
    private DoubleSupplier m_padX, m_padY;
    private double m_deadband = 0.3;
    private boolean m_fieldRelativeTeleop = true;

    // Constructor
    private SwerveDrive() {
        m_gyro.reset();

        SmartDashboard.putData("Field", m_field);
        SmartDashboard.putBoolean("FieldRelativeTeleop", m_fieldRelativeTeleop);
        SmartDashboard.putNumber("DeadZone", m_deadband);
        SmartDashboard.putNumber("Gyro", m_gyro.getRotation2d().getDegrees());

        m_odometry = new SwerveDriveOdometry(
                m_kinematics, m_gyro.getRotation2d(),
                getSwerveModulePositions(), new Pose2d(0, 0, new Rotation2d()));

        RobotConfig config;
        try
        {
            config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                this::getPose, 
                this::resetPose, 
                this::getSpeeds,
                // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative,
                new PPHolonomicDriveController(
                    new PIDConstants(5.0, 0.0, 0.0),
                    // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0)
                    // Rotation PID constants
                ),
                config,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this
            );
        } catch (Exception e)
        {
            e.printStackTrace();
        }
    }

    public void setJoystickSuppliers(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier omegaInput) {
        m_translationX = xInput;
        m_translationY = yInput;
        m_rotationOmega = omegaInput;
    }

    public void setDPadSuppliers(DoubleSupplier xInput, DoubleSupplier yInput) {
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
    public void periodic() {
        boolean FRT = SmartDashboard.getBoolean("FieldRelativeTeleop", true);
        if (FRT != m_fieldRelativeTeleop) {
            m_fieldRelativeTeleop = FRT;
        }
        double DZ = SmartDashboard.getNumber("DeadBand_Drift", 0);
        if (DZ != m_deadband) {
            m_deadband = DZ;
        }
        SmartDashboard.putNumber("Gyro", m_gyro.getRotation2d().getDegrees());
        SmartDashboard.putBoolean("DriverJoystick", isJoystickInputPresent());
        SmartDashboard.putBoolean("DriverDPad", isDPadInputPresent());

        runState();

        m_odometry.update(m_gyro.getRotation2d(), getSwerveModulePositions());
        m_field.setRobotPose(m_odometry.getPoseMeters());
    }

    private void runState() {
        Command currentDriveCommand = null;
        if (!m_state.equals(m_lastState) || isJoystickInputPresent() || isDPadInputPresent()) {
            SmartDashboard.putString("SwerveDriveState", m_state.name());
            switch (m_state) {
                case JOYSTICKS:
                    currentDriveCommand = teleopDrive()
                            .until(() -> !isJoystickInputPresent())
                            .finallyDo((interrupted) -> {
                                if (!interrupted)
                                    setState(SwerveDriveState.IDLE).schedule();
                            });
                    break;
                case D_PAD:
                    currentDriveCommand = dPadDrive()
                            .until(() -> !isDPadInputPresent())
                            .finallyDo((interrupted) -> {
                                if (!interrupted)
                                    setState(SwerveDriveState.IDLE).schedule();
                            });
                    break;
                case IDLE:
                    currentDriveCommand = idleDrive().repeatedly()
                            .until(() -> isJoystickInputPresent() || isDPadInputPresent())
                            .finallyDo((interrupted) -> {
                                if (!interrupted) {
                                    if (isJoystickInputPresent())
                                        setState(SwerveDriveState.JOYSTICKS).schedule();
                                    else if (isDPadInputPresent())
                                        setState(SwerveDriveState.D_PAD).schedule();
                                }
                            });
                    break;
                case LOCKED:
                    break;
                case ON_THE_FLY:
                    break;
                case AUTO:
                    break;
                default:
                    m_state = SwerveDriveState.IDLE;
                    break;
            }

            m_lastState = m_state;

            if (currentDriveCommand != null) {
                currentDriveCommand.schedule();
            }
        }
    }

    public Command setState(SwerveDriveState state) {
        return Commands.runOnce(() -> m_state = state);
    }

    public SwerveDriveState getState() {
        return m_state;
    }

    private Command idleDrive() {
        return Commands.runOnce(() -> {
            m_chassisSpeeds = new ChassisSpeeds();
            setSwerveModuleStates(m_chassisSpeeds);
        }, this);
    }

    private Command lockedDrive() {
        return Commands.runOnce(() -> {
            SwerveModuleState[] lockedStates = new SwerveModuleState[4];

            lockedStates[0] = new SwerveModuleState(0.0, Rotation2d.fromDegrees(45));
            lockedStates[1] = new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45));
            lockedStates[2] = new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45));
            lockedStates[3] = new SwerveModuleState(0.0, Rotation2d.fromDegrees(45));
    
            m_swerveModule1.setDesiredState(lockedStates[0]);
            m_swerveModule2.setDesiredState(lockedStates[1]);
            m_swerveModule3.setDesiredState(lockedStates[2]);
            m_swerveModule4.setDesiredState(lockedStates[3]);

            m_chassisSpeeds = new ChassisSpeeds();
        }, this);
    }

    private Command teleopDrive() {
        return new RunCommand(() -> {
            double X = MathUtil.applyDeadband(m_translationX.getAsDouble(), m_deadband)
                    * SwerveConstants.kDriveMaxSpeed;
            double Y = MathUtil.applyDeadband(m_translationY.getAsDouble(), m_deadband)
                    * SwerveConstants.kDriveMaxSpeed;
            double Omega = MathUtil.applyDeadband(m_rotationOmega.getAsDouble(), m_deadband)
                    * SwerveConstants.kTurnMaxSpeed;

            setSwerveModuleStates(
                drive(X, Y, Omega, m_fieldRelativeTeleop, 0.02)
            );
        }, this);
    }

    private Command dPadDrive() {
        return new RunCommand(() -> {
            double X = m_padX.getAsDouble() * SwerveConstants.kDriveMaxSpeed * 0.3;
            double Y = m_padY.getAsDouble() * SwerveConstants.kDriveMaxSpeed * 0.3;

            setSwerveModuleStates(
                drive(X, Y, 0d, false, 0.02)
            );
        }, this);
    }

    private ChassisSpeeds drive(double x, double y, double omega, boolean fieldRelative, double periodSenconds) {
        m_chassisSpeeds = ChassisSpeeds.discretize(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(x, y, omega, m_gyro.getRotation2d())
                : new ChassisSpeeds(x, y, omega),
            periodSenconds);

        return m_chassisSpeeds;
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

    private void setSwerveModuleStates(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.kDriveMaxSpeed);
        m_swerveModule1.setDesiredState(swerveModuleStates[0]);
        m_swerveModule2.setDesiredState(swerveModuleStates[1]);
        m_swerveModule3.setDesiredState(swerveModuleStates[2]);
        m_swerveModule4.setDesiredState(swerveModuleStates[3]);
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void resetPose(Pose2d p) {
        m_odometry.resetPosition(m_gyro.getRotation2d(), getSwerveModulePositions(), p);
    }

    public ChassisSpeeds getSpeeds() {
        return m_kinematics.toChassisSpeeds(
            m_swerveModule1.getState(),
            m_swerveModule2.getState(),
            m_swerveModule3.getState(),
            m_swerveModule4.getState()
        );
    }

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.kDriveMaxSpeed);
        m_swerveModule1.setDesiredState(swerveModuleStates[0]);
        m_swerveModule2.setDesiredState(swerveModuleStates[1]);
        m_swerveModule3.setDesiredState(swerveModuleStates[2]);
        m_swerveModule4.setDesiredState(swerveModuleStates[3]);
    }

        // ---------------------- NEW CHANGES ------------------------

    // Drive using simple percents in robot frame (forward and rotate), not field-relative
    public void driveVisionPercent(double fwdPercent, double omegaPercent) {
        double X = MathUtil.clamp(fwdPercent, -1.0, 1.0) * SwerveConstants.kDriveMaxSpeed;
        double O = MathUtil.clamp(omegaPercent, -1.0, 1.0) * SwerveConstants.kTurnMaxSpeed;
        setSwerveModuleStates(drive(X, 0.0, O, false, 0.02)); // false = robot-relative
    }

    // Returns true if the driver is moving the sticks past a custom deadband (harder than normal to avoid drift)
    public boolean driverOverrideActive(double overrideDeadband) {
        // Use the same suppliers you already set in RobotContainer
        double x = MathUtil.applyDeadband(m_translationX.getAsDouble(), overrideDeadband);
        double y = MathUtil.applyDeadband(m_translationY.getAsDouble(), overrideDeadband);
        double o = MathUtil.applyDeadband(m_rotationOmega.getAsDouble(), overrideDeadband);
        return (x != 0.0) || (y != 0.0) || (o != 0.0);
}

    // Stop all motion in the drivetrain
    public void stop() {
        setSwerveModuleStates(new ChassisSpeeds());
    }

}
