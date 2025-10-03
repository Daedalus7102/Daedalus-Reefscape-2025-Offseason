package frc.robot.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
  private final NetworkTable table;

  // Use the Limelight's NetworkTables name, usually "limelight"
  public Limelight(String tableName) {
    table = NetworkTableInstance.getDefault().getTable(tableName);
  }

  // 1 = target detected, 0 = no target
  public boolean hasTarget() {
    return table.getEntry("tv").getDouble(0.0) == 1.0;
  }

  // Horizontal offset in degrees (left negative, right positive)
  public double getTx() {
    return table.getEntry("tx").getDouble(0.0);
  }

  // Target area (0..100). Bigger = closer
  public double getTa() {
    return table.getEntry("ta").getDouble(0.0);
  }

  // Select a pipeline you configured for coral
  public void setPipeline(int index) {
    table.getEntry("pipeline").setNumber(index);
  }

  // Turn Limelight LEDs on (3) or off (1)
  public void setLEDOn(boolean on) {
    table.getEntry("ledMode").setNumber(on ? 3 : 1);
  }
}
