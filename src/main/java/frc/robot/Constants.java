package frc.robot;

import frc.robot.subsystems.drive.SwerveModule;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class Constants {
    // Drive Constants
    public static class SwerveConstants {
        // Chassis Constraints
        private static final double kWheelToWheelDistance = 0.6159501; // Distance between wheels in meters

        // Module Constants
        public static final double kWheelDiameter = 0.101; // Wheel diameter in meters
        public static final double kWheelCircumference = Math.PI * kWheelDiameter;
        public static final double kDriveMaxSpeed = 2.2; // Maximum drive speed in meters per second
        public static final double kDriveMaxAcc = 0.5; // Maximum drive acceleration in meters per second squared
        public static final double kDriveGearRatio = 1d/6.75d; // 5.36;
        public static final double kTurnMaxSpeed = Math.PI*2; // Maximum turn speed in radians per second
        public static final double kTurnMaxAcc = 0.15; // Maximum turn acceleration in degrees per second squared
        public static final double kTurnGearRatio = 150d/7d;

        // Drive encoder
        public static final double kDriveVelocityFactor = kWheelCircumference / 60.0; // m/s per RPM

        // Turning encoder
        public static final double kTurnPositionFactor = 360.0 / kTurnGearRatio; // deg / motor rot
        public static final double kTurnVelocityFactor = 1; //kTurnPositionFactor / 60.0; // rad/s per RPM
        // Swerve Drive Kinematics
        public final static SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelToWheelDistance / 2d, kWheelToWheelDistance / 2d), // Frront Left Module
                new Translation2d(kWheelToWheelDistance / 2d, -kWheelToWheelDistance / 2d), // Front Right Module
                new Translation2d(-kWheelToWheelDistance / 2d, kWheelToWheelDistance / 2d), // Back Left Module
                new Translation2d(-kWheelToWheelDistance / 2d, -kWheelToWheelDistance / 2d) // Back Right Module
        );
        // Swerve Modules

        //    .---.         .---.   Forward = 0°
        //    | 1 |▩▩▩▩▩▩▩▩▩| 2 |
        //    '---'         '---'    (x+)|
        //      █     /°\     █          |
        //      █      |      █   (y+)---|---(y-)
        //      █      |      █          |
        //    .---.         .---.        |(x-)
        //    | 3 |▩▩▩▩▩▩▩▩▩| 4 |        
        //    '---'         '---'

        public static final SwerveModule kFrontLeft = new SwerveModule(5, 6, 1, 0.051758, false);
        public static final SwerveModule kFrontRight = new SwerveModule(7, 8, 2, 0.434570, true);
        public static final SwerveModule kBackLeft = new SwerveModule(9, 10, 3, -0.047363, false);
        public static final SwerveModule kBackRight = new SwerveModule(11, 12, 4, 0.065674, true);

        // Drive Motor PID Constants
        public static final double kDriveP = 0.02, kDriveI = 0.0, kDriveD = 0.0, kDriveFF = 0.3;

        // Turning Motor PID Constants
        public static final double kTurnP = 0.01, kTurnI = 0.0, kTurnD = 0.0;

        // Drive Motor Configuration
        public static final int kDriveCurrentLimitA = 40;
        public static final double kDriveVoltageComp = 12.0;
        public static final double kDriveOpenLoopRamp = 0.5;
        public static final double kDriveClosedLoopRamp = 0.5;

        // Turning Motor Configuration
        public static final int kTurnCurrentLimitA = 30;
        public static final double kTurnVoltageComp = 12.0;
        public static final double kTurnOpenLoopRamp = 0.15;
        public static final double kTurnClosedLoopRamp = 0.15;

        // CAN bus
        public static final String kCANbus = "rio";

        // Pigeon ID
        public static final int kPigeonID = 13;
    }

    public static class ElevatorConstants {
        // Motor IDs
        public static final int kElevatorMotorID = 14;
        public static final int kFollowerMotorID = 15;

        // Elevator Heights
        public enum ElevatorHeight {
            ZERO(0),
            L1(10),
            L2(30),
            P1(50),
            L3(70),
            P2(90),
            L4(110),
            NET(130);

            private final double height;
            ElevatorHeight(double height) {
                this.height = height;
            }

            public double getHeight() {
                return height;
            }

            public ElevatorHeight next() {
                ElevatorHeight[] values = values();
                int index = this.ordinal();
                return index < values.length - 1 ? values[index + 1] : this;
            }
            public ElevatorHeight previous() {
                int index = this.ordinal();
                return index > 0 ? values()[index - 1] : this;
            }
    
            public boolean greaterThan(ElevatorHeight other) {
                return this.height > other.height;
            }
        }

        // Elevator Configuration
        public static final int kElevatorCurrentLimitA = 40;

        // Elevator PID Constants
        public static final double kElevatorP = 1.0, kElevatorI = 0.0, kElevatorD = 0.0, kElevatorFF = 0.0;

        // Soft Limits
        public static final double kElevatorMinHeight = 0;
        public static final double kElevatorMaxHeight = 130;
    }

    /* =========================
                Intake
       ========================= */
    public static final class IntakeConstants {
        // CAN IDs
        public static final int kPivotMotorID  = 30; // TODO: set actual ID
        public static final int kRollerMotorID = 31; // TODO: set actual ID

        // Inversions
        public static final boolean kPivotInverted  = false; // TODO: invert if needed
        public static final boolean kRollerInverted = false; // TODO: invert if needed

        // Current Limits
        public static final int kPivotCurrentLimitA  = 40;
        public static final int kRollerCurrentLimitA = 40;

        // PID (pivot position hold)
        public static final double kPivotP = 0.1;
        public static final double kPivotI = 0.0;
        public static final double kPivotD = 0.0;
        public static final double kPivotFF = 0.0;

        // Units and soft limits
        public static final double kPivotPosConversion = 360.0; // TODO: set to gearbox ratio
        public static final boolean kUsePivotSoftLimits = true;
        public static final float kPivotForwardLimitRot = 0.0f;   // Stowed side (greater angle)
        public static final float kPivotReverseLimitRot = -1.5f;  // Floor side (smaller angle) TODO tune

        // Named setpoints (rotations, after conversion)
        public static final double kPivotStowRot  = -0.1; // TODO tune
        public static final double kPivotFloorRot = -1.4; // TODO tune

        // Roller speeds
        public static final double kRollerInPercent  = 0.85;
        public static final double kRollerOutPercent = -0.6;
    }

    // Vision constants (Limelight name)
    public static final class VisionConstants {
        public static final String kLimelightName = "limelight"; // Change if you renamed it (e.g., "limelight-coral")
    }

    // AutoIntake (used by AutoTake)
    public static final class AutoIntakeConstants {
        public static final int kCoralPipeline = 0;           // Limelight pipeline for coral
        public static final double kDriverOverrideDB = 0.50;  // Driver must push past this to cancel AutoTake
        public static final double kTurnKp = 0.03;            // PID Kp for turning using tx
        public static final double kMaxTurn = 0.7;            // Clamp for turn command
        public static final double kFwdKp = 0.30;             // Forward proportional on area error
        public static final double kMaxFwd = 0.8;             // Clamp for forward command
        public static final double kCloseArea = 6.0;          // Stop/hold when ta >= this
        public static final double kLostTimeoutS = 0.50;      // End if target lost for this time
        public static final double kCaptureHoldS = 0.35;      // Time to keep rollers running after close
        public static final boolean kAutoStowAfterCapture = true; // Set false to keep intake down after capture
    }
}