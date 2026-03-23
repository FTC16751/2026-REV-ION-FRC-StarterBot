// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ShooterSubsystemConstants.FeederSetpoints;
import frc.robot.Constants.ShooterSubsystemConstants.FlywheelSetpoints;

/** Shooter subsystem that delegates hardware access to a ShooterIO implementation. */
public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  // The annotation processor will generate ShooterIOInputsAutoLogged
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  // Mutable shoot speed — adjusted by operator X/B buttons (manual) or table (auto)
  private double currentShootRpm = FlywheelSetpoints.kShootRpm;
  // Toggle for auto (distance-based) vs manual speed mode
  private boolean autoSpeedMode = false;

  // Supplier of distance-to-goal in meters (wired to Drive.distanceToTarget() in RobotContainer)
  private final DoubleSupplier distanceToTargetMeters;

  // Distance (meters) → RPM lookup table. Values between entries are linearly interpolated.
  // *** PLACEHOLDER VALUES — tune these on the field! ***
  private static final InterpolatingDoubleTreeMap shooterRpmTable = new InterpolatingDoubleTreeMap();
  static {
    shooterRpmTable.put(1.5, 3500.0); // close range  (~5 ft)
    shooterRpmTable.put(2.5, 4500.0); // medium range (~8 ft)
    shooterRpmTable.put(3.5, 5200.0); // far range    (~11 ft)
    shooterRpmTable.put(5.0, 5500.0); // max effective range (~16 ft)
  }

  /**
   * @param io Hardware/sim IO implementation
   * @param distanceToTargetMeters Supplier of robot-to-goal distance in meters (used in auto mode)
   */
  public Shooter(ShooterIO io, DoubleSupplier distanceToTargetMeters) {
    this.io = io;
    this.distanceToTargetMeters = distanceToTargetMeters;
    System.out.println("---> Shooter initialized (IO based)");
  }

  /** Constructor with no distance supplier — auto mode will hold last manual RPM. */
  public Shooter(ShooterIO io) {
    this(io, () -> 0.0);
  }

  /** Convenience constructor: constructs a hardware IO implementation. */
  public Shooter() {
    this(new ShooterIOSpark(), () -> 0.0);
  }

  private boolean isFlywheelAt(double velocity) {
    return MathUtil.isNear(inputs.flywheelVelocity, velocity, FlywheelSetpoints.kVelocityTolerance);
  }

  public final Trigger isFlywheelSpinning = new Trigger(
      () -> isFlywheelAt(currentShootRpm) || inputs.flywheelVelocity >= currentShootRpm
  );

  public final Trigger isFlywheelSpinningBackwards = new Trigger(
      () -> isFlywheelAt(-currentShootRpm) || inputs.flywheelVelocity <= -currentShootRpm
  );

  public final Trigger isFlywheelStopped = new Trigger(() -> isFlywheelAt(0));

  private void setFlywheelVelocity(double velocity) {
    io.setFlywheelVelocity(velocity);
  }

  private void setFeederPower(double power) {
    io.setFeederPower(power);
  }

  public Command runFlywheelCommand() {
    return this.startEnd(
        () -> { this.setFlywheelVelocity(currentShootRpm); },
        () -> { this.setFlywheelVelocity(0.0); }
    ).withName("Spinning Up Flywheel");
  }

  public Command runFeederCommand() {
    return this.startEnd(
        () -> {
          this.setFlywheelVelocity(currentShootRpm);
          this.setFeederPower(FeederSetpoints.kFeed);
        },
        () -> {
          this.setFlywheelVelocity(0.0);
          this.setFeederPower(0.0);
        }
    ).withName("Feeding");
  }

  /** Runs feeder motor only (no flywheel). Use for manual D-Pad control. */
  public Command runFeederOnlyCommand() {
    return this.startEnd(
        () -> this.setFeederPower(FeederSetpoints.kFeed),
        () -> this.setFeederPower(0.0)
    ).withName("Feeder Forward");
  }

  /** Runs feeder motor in reverse only. */
  public Command runFeederReverseCommand() {
    return this.startEnd(
        () -> this.setFeederPower(-FeederSetpoints.kFeed),
        () -> this.setFeederPower(0.0)
    ).withName("Feeder Reverse");
  }

  /** Adjusts currentShootRpm by deltaRpm (e.g. +100 or -100), clamped to a safe range.
   *  Only applies in manual mode; ignored when autoSpeedMode is active. */
  public Command adjustShooterSpeedCommand(double deltaRpm) {
    return Commands.runOnce(() -> {
      if (!autoSpeedMode) {
        currentShootRpm = MathUtil.clamp(
            currentShootRpm + deltaRpm,
            0,
            FlywheelSetpoints.kMaxRpm);
      }
    }).withName(String.format("Adjust Shooter RPM %+.0f", deltaRpm));
  }

  /** Toggles between auto (distance-based table) and manual speed mode.
   *  Switching back to manual restores the default kShootRpm preset. */
  public Command toggleAutoSpeedCommand() {
    return Commands.runOnce(() -> {
      autoSpeedMode = !autoSpeedMode;
      if (!autoSpeedMode) {
        // Reset to the constant preset when returning to manual
        currentShootRpm = FlywheelSetpoints.kShootRpm;
      }
    }).withName("Toggle Auto Shooter Speed");
  }

  public Command runShooterCommand() {
    return this.run(() -> this.setFlywheelVelocity(currentShootRpm))
        .until(isFlywheelSpinning)
        .finallyDo(interrupted -> { if (interrupted) io.stop(); })
        .andThen(
            // Use run() (not startEnd) so currentShootRpm changes from X/B buttons
            // take effect every cycle while the shooter is active.
            this.run(() -> {
              this.setFlywheelVelocity(currentShootRpm);
              this.setFeederPower(FeederSetpoints.kFeed);
            }).finallyDo(() -> io.stop())
        ).withName("Shooting");
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    // In auto mode, update currentShootRpm from the distance→RPM table each cycle
    if (autoSpeedMode) {
      double distMeters = distanceToTargetMeters.getAsDouble();
      currentShootRpm = MathUtil.clamp(
          shooterRpmTable.get(distMeters),
          0,
          FlywheelSetpoints.kMaxRpm);
    }

    Logger.processInputs("Shooter", inputs);
    Logger.recordOutput("Shooter/currentShootRpm", currentShootRpm);
    Logger.recordOutput("Shooter/autoSpeedMode", autoSpeedMode);
    Logger.recordOutput("Shooter/distanceToTargetMeters", distanceToTargetMeters.getAsDouble());
  }
}
