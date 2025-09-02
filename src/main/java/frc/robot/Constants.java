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
        public static final double kDriveMaxSpeed = 2.0; // Maximum drive speed in meters per second
        public static final double kDriveMaxAcc = 0.5; // Maximum drive acceleration in meters per second squared
        public static final double kDriveGearRatio = 6.75; // 5.36;
        public static final double kTurnMaxSpeed = Math.PI / 2; // Maximum turn speed in radians per second
        public static final double kTurnMaxAcc = 0.15; // Maximum turn acceleration in degrees per second squared
        public static final double kTurnGearRatio = 150 / 7; // 18.75;
        // Drive encoder
        public static final double kDriveVelocityFactor = kWheelCircumference / 60.0; // m/s per RPM
        // Turning encoder
        public static final double kTurnPositionFactor = 360 / kTurnGearRatio; // deg / motor rot
        public static final double kTurnVelocityFactor = 1; //kTurnPositionFactor / 60.0; // rad/s per RPM
        // Swerve Drive Kinematics
        public final static SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelToWheelDistance / 2, kWheelToWheelDistance / 2), // Frront Left Module
                new Translation2d(kWheelToWheelDistance / 2, -kWheelToWheelDistance / 2), // Front Right Module
                new Translation2d(-kWheelToWheelDistance / 2, kWheelToWheelDistance / 2), // Back Left Module
                new Translation2d(-kWheelToWheelDistance / 2, -kWheelToWheelDistance / 2) // Back Right Module
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

        public static final SwerveModule kFrontLeft = new SwerveModule(5, 6, 1, -0.452148, true);
        public static final SwerveModule kFrontRight = new SwerveModule(7, 8, 2, -0.064453, false);
        public static final SwerveModule kBackLeft = new SwerveModule(9, 10, 3, 0.446533, true);
        public static final SwerveModule kBackRight = new SwerveModule(11, 12, 4, 0.072998, false);
        // Drive Motor PID Constants
        public static final double kDriveP = 0.06, kDriveI = 0.0, kDriveD = 0.0, kDriveFF = 0.2;
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
    }
}
