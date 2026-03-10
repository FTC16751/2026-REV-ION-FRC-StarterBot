// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for the Shooter so we can plug in a hardware implementation or a sim implementation
 * and record/replay inputs using AutoLog.
 */
public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    // Flywheel
    public double flywheelAppliedVoltage = 0.0;
    public double flywheelVelocity = 0.0; // RPM (matches REV encoder getVelocity output)
    public double flywheelTargetVelocity = 0.0;
    public double flywheelCurrent = 0.0;
    public boolean flywheelConnected = true;
    public double flyFollowerAppliedVoltage = 0.0;
    public boolean flyFollowerConnected = true;

    // Feeder
    public double feederVelocity = 0.0; // RPM (matches REV encoder getVelocity output)
    public double feederAppliedVoltage = 0.0;
    public double feederCurrent = 0.0;
    public boolean feederConnected = true;
  }

  /** Update the inputs structure (called from subsystem periodic). */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Set flywheel target velocity in RPM (uses closed-loop controller in hardware). */
  public default void setFlywheelVelocity(double rpm) {}

  /** Set feeder motor output in [-1,1]. */
  public default void setFeederPower(double power) {}

  /** Stop both shooter outputs. */
  public default void stop() {}
}
