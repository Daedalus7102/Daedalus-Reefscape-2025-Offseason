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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorHeight;

public class Elevator extends SubsystemBase {

    // Singleton instance
    private static Elevator m_instance = null;

    public static Elevator getInstance() {
        if (m_instance == null) {
            m_instance = new Elevator();
        }
        return m_instance;
    }

    // Elevator State
    public enum ElevatorState{
        UP,
        HOLD,
        DOWN,
    }

    private ElevatorState m_state, m_lastState;

    // Elevator Height
    private ElevatorHeight m_elevatorHeight;

    // Motors
    private SparkMax m_elevatorMotor, m_followerMotor;
    private SparkMaxConfig m_elevatorMotorConfig, m_followerMotorConfig;

    private SparkClosedLoopController m_closedLoopController;
    private RelativeEncoder m_turningEncoder;

    // Class Constructor
    public Elevator() {
        m_elevatorMotor = new SparkMax(ElevatorConstants.kElevatorMotorID, MotorType.kBrushless);
        m_followerMotor = new SparkMax(ElevatorConstants.kFollowerMotorID, MotorType.kBrushless);

        m_closedLoopController = m_elevatorMotor.getClosedLoopController();
        m_turningEncoder = m_elevatorMotor.getEncoder();

        m_elevatorMotorConfig = new SparkMaxConfig();
        m_elevatorMotorConfig.inverted(true);
        m_elevatorMotorConfig.idleMode(IdleMode.kBrake);
        m_elevatorMotorConfig.smartCurrentLimit(ElevatorConstants.kElevatorCurrentLimitA);
        m_elevatorMotorConfig.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);
        m_elevatorMotorConfig.softLimit
            .forwardSoftLimit(ElevatorConstants.kElevatorMaxHeight)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(ElevatorConstants.kElevatorMinHeight)
            .reverseSoftLimitEnabled(true);
        m_elevatorMotorConfig.closedLoopRampRate(0);
        m_elevatorMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(ElevatorConstants.kElevatorP)
            .i(ElevatorConstants.kElevatorI)
            .d(ElevatorConstants.kElevatorD)
            .velocityFF(ElevatorConstants.kElevatorFF)
            .outputRange(-1, 1);

        m_followerMotorConfig = new SparkMaxConfig();
        m_followerMotorConfig.idleMode(IdleMode.kBrake);
        m_followerMotorConfig.smartCurrentLimit(ElevatorConstants.kElevatorCurrentLimitA);
        m_followerMotorConfig.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);
        m_followerMotorConfig.follow(ElevatorConstants.kElevatorMotorID, true);

        m_elevatorMotor.configure(m_elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        m_followerMotor.configure(m_followerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        m_state =  ElevatorState.HOLD;
        m_elevatorHeight = ElevatorHeight.ZERO;
    }

    public ElevatorHeight getElevatorHeight(){
        return m_elevatorHeight;
    }

    @Override
    public void periodic(){
        SmartDashboard.putString("ElevatorHeight", m_elevatorHeight.name());
    }

    public Command moveElevatorTo(ElevatorHeight desiredHeight){
        double target_height = desiredHeight.getHeight();
        return new RunCommand(() -> {
            m_closedLoopController.setReference(target_height, ControlType.kPosition);
            m_elevatorHeight = desiredHeight;
        },this).until(() -> (Math.abs(target_height-m_turningEncoder.getPosition()) < 1.5)).finallyDo(() -> m_elevatorHeight = desiredHeight);
    }

    public Command moveElevator(double speed){
        return Commands.runOnce(() -> {
            m_elevatorMotor.set(speed);
        }, this);
    }

    public Command stopElevator(){
        return Commands.runOnce(() -> {
            m_elevatorMotor.stopMotor();
        }, this);
    }
    
}
