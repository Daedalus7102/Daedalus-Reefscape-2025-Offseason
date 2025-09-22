package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;

public class Intake extends edu.wpi.first.wpilibj2.command.SubsystemBase {

    private final SparkMax m_pivotMotor = new SparkMax(Constants.IntakeConstants.kPivotMotorID, MotorType.kBrushless);
    private final SparkMax m_rollerMotor = new SparkMax(Constants.IntakeConstants.kRollerMotorID, MotorType.kBrushless);

    private final RelativeEncoder m_pivotEncoder = m_pivotMotor.getEncoder();
    private final SparkClosedLoopController m_pivotPID = m_pivotMotor.getClosedLoopController();

    public Intake() {
        // Pivot config
        var pivotCfg = new SparkMaxConfig();
        pivotCfg.idleMode(IdleMode.kBrake);
        pivotCfg.smartCurrentLimit(Constants.IntakeConstants.kPivotCurrentLimitA);
        pivotCfg.inverted(Constants.IntakeConstants.kPivotInverted);
        pivotCfg.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(Constants.IntakeConstants.kPivotP)
            .i(Constants.IntakeConstants.kPivotI)
            .d(Constants.IntakeConstants.kPivotD)
            .velocityFF(Constants.IntakeConstants.kPivotFF);
        if (Constants.IntakeConstants.kUsePivotSoftLimits) {
            pivotCfg.softLimit.forwardSoftLimit(Constants.IntakeConstants.kPivotForwardLimitRot)
                    .forwardSoftLimitEnabled(true);
            pivotCfg.softLimit.reverseSoftLimit(Constants.IntakeConstants.kPivotReverseLimitRot)
                    .reverseSoftLimitEnabled(true);
        }
        // Set conversion for rotations
        pivotCfg.encoder.positionConversionFactor(Constants.IntakeConstants.kPivotPosConversion);
        m_pivotMotor.configure(pivotCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Roller config
        var rollerCfg = new SparkMaxConfig();
        rollerCfg.idleMode(IdleMode.kCoast);
        rollerCfg.smartCurrentLimit(Constants.IntakeConstants.kRollerCurrentLimitA);
        rollerCfg.inverted(Constants.IntakeConstants.kRollerInverted);
        m_rollerMotor.configure(rollerCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SmartDashboard.putBoolean("Intake/UseSoftLimits", Constants.IntakeConstants.kUsePivotSoftLimits);
    }

    // -------- Low-level controls --------
    public void setPivotPercent(double percent) {
        m_pivotMotor.set(percent);
    }

    // Hold a pivot setpoint in *rotations* (after conversion factor).
    public void holdPivotSetpoint(double rotations) {
        m_pivotPID.setReference(rotations, ControlType.kPosition);
    }

    // Run roller with percent [-1,1]. Positive = intake by convention.
    public void setRollerPercent(double percent) {
        m_rollerMotor.set(percent);
    }

    public void stopAll() {
        m_pivotMotor.stopMotor();
        m_rollerMotor.stopMotor();
    }

    public double getPivotPosition() {
        return m_pivotEncoder.getPosition(); // rotations (after conversion factor)
    }

    // -------- Command helpers (easy to bind) --------
    public Command pivotManual(double percent) {
        return Commands.run(() -> setPivotPercent(percent), this)
                       .finallyDo(interrupted -> setPivotPercent(0.0));
    }

    public Command pivotToStow() {
        return Commands.run(() -> holdPivotSetpoint(Constants.IntakeConstants.kPivotStowRot), this);
    }

    public Command pivotToFloor() {
        return Commands.run(() -> holdPivotSetpoint(Constants.IntakeConstants.kPivotFloorRot), this);
    }

    public Command rollerIn() {
        return Commands.run(() -> setRollerPercent(Constants.IntakeConstants.kRollerInPercent), this)
                       .finallyDo(i -> setRollerPercent(0.0));
    }

    public Command rollerOut() {
        return Commands.run(() -> setRollerPercent(Constants.IntakeConstants.kRollerOutPercent), this)
                       .finallyDo(i -> setRollerPercent(0.0));
    }

    public Command rollerStop() {
        return Commands.runOnce(() -> setRollerPercent(0.0), this);
    }

    public Command stop() {
        return Commands.runOnce(this::stopAll, this);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/PivotRot", getPivotPosition());
    }
}
