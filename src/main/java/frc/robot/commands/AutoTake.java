package frc.robot.commands;

import static frc.robot.Constants.AutoIntakeConstants.*;
import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.vision.Limelight;

public class AutoTake extends Command {
  private final SwerveDrive drive;
  private final Intake intake;
  private final Limelight limelight;

  // Turn PID uses KP only (simple proportional on tx)
  private final PIDController turnPID = new PIDController(kTurnKp, 0, 0);

  private double lastSeenTime = 0.0;          // Last time Limelight saw a valid target
  private Double captureStartTime = null;     // When ta first reached kCloseArea

  public AutoTake(SwerveDrive drive, Intake intake, Limelight limelight) {
    this.drive = drive;
    this.intake = intake;
    this.limelight = limelight;
    addRequirements(drive); // Requires drivetrain so manual control yields while held
  }

  @Override
  public void initialize() {
    limelight.setPipeline(kCoralPipeline); // Switch LL to coral pipeline
    limelight.setLEDOn(true);              // Turn LEDs on
    turnPID.setSetpoint(0.0);              // We want tx -> 0
    lastSeenTime = Timer.getFPGATimestamp();
    captureStartTime = null;

    // Drop intake to floor and start rollers inward (these return Command, so schedule them)
    intake.pivotToFloor().schedule();
    intake.rollerIn().schedule();
  }

  @Override
  public void execute() {
    // Driver override: if driver moves sticks past stricter deadband, end immediately
    if (drive.driverOverrideActive(kDriverOverrideDB)) {
      return; // isFinished() will return true this loop, end() will run cleanup
    }

    final double now = Timer.getFPGATimestamp();

    if (limelight.hasTarget()) {
      lastSeenTime = now;

      final double tx = limelight.getTx();
      final double ta = limelight.getTa();

      // If close enough, start capture hold and stop moving
      if (ta >= kCloseArea) {
        if (captureStartTime == null) captureStartTime = now; // Start hold window once
        drive.stop(); // Stop drive while rollers keep pulling the coral
        return;
      }

      // Turn command from tx
      double omegaCmd = MathUtil.clamp(turnPID.calculate(tx, 0.0), -kMaxTurn, kMaxTurn);

      // Forward command from how far we are from the target size
      double areaError = (kCloseArea - ta);
      double fwdCmd = MathUtil.clamp(kFwdKp * areaError, -kMaxFwd, kMaxFwd);

      // Do not back away automatically
      fwdCmd = MathUtil.clamp(fwdCmd, 0.0, kMaxFwd);

      // Robot-relative driving for vision chasing
      drive.driveVisionPercent(fwdCmd, omegaCmd);
    } else {
      // Hold still to let driver re-aim
      drive.stop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    limelight.setLEDOn(false);

    // Stop rollers
    intake.rollerStop().schedule();

    // Optional auto-stow behavior controlled by constant
    if (kAutoStowAfterCapture) {
      intake.pivotToStow().schedule();
    }
  }

  @Override
  public boolean isFinished() {
    // Cancel immediately if driver moves the sticks past override deadband
    if (drive.driverOverrideActive(kDriverOverrideDB)) return true;

    // Finish after holding long enough to secure piece
    final double now = Timer.getFPGATimestamp();
    if (captureStartTime != null && (now - captureStartTime) >= kCaptureHoldS) return true;

    // Or if we have lost the target for too long
    return (now - lastSeenTime) > kLostTimeoutS;
  }
}
  