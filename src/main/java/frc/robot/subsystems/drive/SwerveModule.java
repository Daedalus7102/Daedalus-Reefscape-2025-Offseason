package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
    // Module
    private SwerveModuleState moduleState = new SwerveModuleState();
    // Motors
    private final SparkMax driveMotor;
    private final SparkMaxConfig driveMotorConfig;
    private final RelativeEncoder driveRelativeEncoder;

    private final SparkMax turnMotor;
    private final SparkMaxConfig turnMotorConfig;
    private final RelativeEncoder turnRelativeEncoder;
    // Absolute Encoder
    private final CANcoder turnEncoder;
    private final CANcoderConfiguration turnEncoderConfig;
    private final double offset;

    
    // Class Constructor
    public SwerveModule(int driveMotorID, int turnMotorID, int CANcoderID , double turnEncoderOffset, boolean isInverted) {
        // Drive Motor
        this.driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
        this.driveRelativeEncoder = driveMotor.getEncoder();
        this.driveMotorConfig = new SparkMaxConfig();
        this.driveMotorConfig
            .smartCurrentLimit(SwerveConstants.kDriveCurrentLimitA)
            .idleMode(SparkMaxConfig.IdleMode.kBrake)
            .inverted(false)
            .voltageCompensation(SwerveConstants.kDriveVoltageComp)
            .openLoopRampRate(SwerveConstants.kDriveOpenLoopRamp)
            .closedLoopRampRate(SwerveConstants.kDriveClosedLoopRamp);
        this.driveMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(SwerveConstants.kDriveP)
            .i(SwerveConstants.kDriveI)
            .d(SwerveConstants.kDriveD)
            .outputRange(-1, 1);
        this.driveMotorConfig.encoder
            .positionConversionFactor(SwerveConstants.kDrivePositionFactor)
            .velocityConversionFactor(SwerveConstants.kDriveVelocityFactor);
        // Turn Motor
        this.turnMotor = new SparkMax(turnMotorID, MotorType.kBrushless);
        this.turnRelativeEncoder = turnMotor.getEncoder();
        this.turnMotorConfig = new SparkMaxConfig();
        this.turnMotorConfig
            .smartCurrentLimit(SwerveConstants.kTurnCurrentLimitA)
            .idleMode(SparkMaxConfig.IdleMode.kBrake)
            .inverted(false)
            .voltageCompensation(SwerveConstants.kTurnVoltageComp)
            .openLoopRampRate(SwerveConstants.kTurnOpenLoopRamp)
            .closedLoopRampRate(SwerveConstants.kTurnClosedLoopRamp);
        this.turnMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(SwerveConstants.kTurnP)
            .i(SwerveConstants.kTurnI)
            .d(SwerveConstants.kTurnD)
            .outputRange(-1, 1);
        this.turnMotorConfig.encoder
            .positionConversionFactor(SwerveConstants.kTurnPositionFactor)
            .velocityConversionFactor(SwerveConstants.kTurnVelocityFactor);
        // Absolute Encoder
        this.offset = turnEncoderOffset;
        this.turnEncoder = new CANcoder(CANcoderID, SwerveConstants.kCANbus);
        this.turnEncoderConfig = new CANcoderConfiguration();
        this.turnEncoderConfig
            .MagnetSensor.MagnetOffset = this.offset;
        this.turnEncoderConfig
            .MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        // Apply Configs
        this.driveMotor.configure(driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        this.turnMotor.configure(turnMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        this.turnEncoder.getConfigurator().apply(turnEncoderConfig);

        syncAzimuthToAbsolute();
    }

    // Brake Configuration
    public void motorsToBrake() {
        driveMotorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        turnMotorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        driveMotor.configure(driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        turnMotor.configure(turnMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // Coast Configuration
    public void motorsToCoast() {
        driveMotorConfig.idleMode(SparkMaxConfig.IdleMode.kCoast);
        turnMotorConfig.idleMode(SparkMaxConfig.IdleMode.kCoast);
        driveMotor.configure(driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        turnMotor.configure(turnMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // Stop Motors
    public void stop(){
        driveMotor.stopMotor();
        turnMotor.stopMotor();
    }

    // Set to Desired State
    public void setDesiredState(SwerveModuleState desired) {
        moduleState = desired;
        var current = Rotation2d.fromRadians(turnRelativeEncoder.getPosition());
        moduleState.optimize(current);

        driveMotor.getClosedLoopController().setReference(
            moduleState.speedMetersPerSecond, 
            SparkMax.ControlType.kVelocity);
        turnMotor.getClosedLoopController().setReference(
            moduleState.angle.getRadians(), 
            SparkMax.ControlType.kPosition);
    }

    // Coordinates Encoder Position
    public final void syncAzimuthToAbsolute() {
        // Phoenix 6: getAbsolutePosition() returns rotations in [0,1)
        double absRot = turnEncoder.getAbsolutePosition().getValueAsDouble();
        double absRad = absRot * 2.0 * Math.PI;

        double moduleRad = Rotation2d.fromRadians(absRad).getRadians();
        turnRelativeEncoder.setPosition(moduleRad);
    }

    public SwerveModulePosition getModulePosition() {
        Rotation2d rotation2d = new Rotation2d(Math.toRadians(driveRelativeEncoder.getPosition()));
        double position = driveRelativeEncoder.getPosition() * SwerveConstants.kWheelDiameter * Math.PI;
        return new SwerveModulePosition(position, rotation2d);
    }

    public void resetModulePosition() {
        driveRelativeEncoder.setPosition(0);
    }

    public SwerveModuleState getState(){
        return moduleState;
    }

}
