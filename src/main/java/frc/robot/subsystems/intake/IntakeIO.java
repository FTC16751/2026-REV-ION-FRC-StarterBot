package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;

/** IO interface for the Intake so we can plug in a hardware implementation or a sim implementation */
public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    // Intake
    public double intakeAppliedVoltage = 0.0;
    public boolean intakeConnected = true;

    // Conveyor
    public double conveyorAppliedVoltage = 0.0;
    public boolean conveyorConnected = true;

    // Pivot
    public double pivotPosition = 0.0; // degrees
    public double pivotVelocity = 0.0; // degrees / sec 
    public double pivotAppliedVoltage = 0.0;
    public double pivotCurrent = 0.0;
    public double pivotTargetPosition = 0.0;
    public boolean pivotConnected = true;
  }

  /** Update the inputs structure (called from subsystem periodic). */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Set intake motor output in [-1,1]. */
  public default void setIntakePower(double power) {}
  
  public default void setIntakeSpeed(AngularVelocity speed) {}

  /** Set conveyor motor output in [-1,1]. */
  public default void setConveyorPower(double power) {}

  /** Set pivot position in degrees (closed-loop in hardware). */
  public default void setPivotPosition(double degrees) {}

  public default void zeroPivotPosition(boolean bottom) {}

  /** Stop all outputs. */
  public default void stop() {}
}
