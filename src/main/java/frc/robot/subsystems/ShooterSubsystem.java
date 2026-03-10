// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/**
 * Legacy compatibility wrapper. The new IO-based shooter is in {@link Shooter}.
 *
 * Keep this thin wrapper so existing code that referenced the old
 * ShooterSubsystem class continues to work. Prefer using {@link Shooter}
 * directly in new code.
 */
@Deprecated
public class ShooterSubsystem extends Shooter {
  public ShooterSubsystem() {
    super();
  }

  public ShooterSubsystem(ShooterIO io) {
    super(io);
  }
}
