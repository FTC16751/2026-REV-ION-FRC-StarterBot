package frc.robot.subsystems.conveyor;

import org.littletonrobotics.junction.AutoLog;

/** IO interface for the Conveyor so we can plug in a hardware or sim implementation */
public interface ConveyorIO {
  @AutoLog
  public static class ConveyorIOInputs {
    public double conveyorAppliedVoltage = 0.0;
    public double conveyorCurrent = 0.0;
    public boolean conveyorConnected = true;
  }

  /** Update the inputs structure (called from subsystem periodic). */
  public default void updateInputs(ConveyorIOInputs inputs) {}

  /** Set conveyor motor output in [-1,1]. */
  public default void setConveyorPower(double power) {}
}