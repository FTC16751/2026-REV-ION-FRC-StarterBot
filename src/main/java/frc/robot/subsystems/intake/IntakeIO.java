package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/** IO interface for the Intake so we can plug in a hardware implementation or a sim implementation */
public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    // Intake
    public double intakeAppliedVoltage = 0.0;
    public boolean intakeConnected = true;

    // Pivot
    public double pivotPosition = 0.0; // absolute encoder position (0.0–1.0 rotations)
    public double pivotVelocity = 0.0; // rotations per second (absolute encoder)
    public double pivotAppliedVoltage = 0.0;
    public double pivotCurrent = 0.0;
    public double pivotTargetPosition = 0.0;
    public boolean pivotConnected = true;
  }

  /** Update the inputs structure (called from subsystem periodic). */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Set intake motor output in [-1,1]. */
  public default void setIntakePower(double power) {}

  /** Set pivot target position as an absolute encoder value (0.0–1.0 rotations). */
  public default void setPivotPosition(double position) {}

  public default void zeroPivotPosition() {}

  /** Set pivot motor output in open-loop duty cycle [-1,1]. Used for compliant hold at target. */
  public default void setPivotDutyCycle(double dutyCycle) {}

  /** Stop all outputs. */
  public default void stop() {}
}
