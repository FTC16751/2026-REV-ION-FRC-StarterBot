// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;
import frc.robot.Configs;
import frc.robot.Constants.ShooterSubsystemConstants.FeederSetpoints;
import frc.robot.Constants.ShooterSubsystemConstants.FlywheelSetpoints;
import frc.robot.Constants.ShooterSubsystemConstants;

/** Shooter subsystem that delegates hardware access to a ShooterIO implementation. */
public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  // The annotation processor will generate ShooterIOInputsAutoLogged
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private double flywheelTargetVelocity = 0.0;

  public Shooter(ShooterIO io) {
    this.io = io;
  System.out.println("---> Shooter initialized (IO based)");
  }

  /** Convenience constructor: constructs a hardware IO implementation. */
  public Shooter() {
    this(new ShooterIOSpark());
  }

  private boolean isFlywheelAt(double velocity) {
    return MathUtil.isNear(inputs.flywheelVelocity, velocity, FlywheelSetpoints.kVelocityTolerance);
  }

  public final Trigger isFlywheelSpinning = new Trigger(
      () -> isFlywheelAt(5000) || inputs.flywheelVelocity > 5000
  );

  public final Trigger isFlywheelSpinningBackwards = new Trigger(
      () -> isFlywheelAt(-5000) || inputs.flywheelVelocity < -5000
  );

  public final Trigger isFlywheelStopped = new Trigger(() -> isFlywheelAt(0));

  private void setFlywheelVelocity(double velocity) {
    io.setFlywheelVelocity(velocity);
    flywheelTargetVelocity = velocity;
  }

  private void setFeederPower(double power) {
    io.setFeederPower(power);
  }

  public Command runFlywheelCommand() {
    return this.startEnd(
        () -> { this.setFlywheelVelocity(FlywheelSetpoints.kShootRpm); },
        () -> { this.setFlywheelVelocity(0.0); }
    ).withName("Spinning Up Flywheel");
  }

  public Command runFeederCommand() {
    return this.startEnd(
        () -> {
          this.setFlywheelVelocity(FlywheelSetpoints.kShootRpm);
          this.setFeederPower(FeederSetpoints.kFeed);
        },
        () -> {
          this.setFlywheelVelocity(0.0);
          this.setFeederPower(0.0);
        }
    ).withName("Feeding");
  }

  public Command runShooterCommand() {
    return this.run(() -> this.setFlywheelVelocity(FlywheelSetpoints.kShootRpm))
        .until(isFlywheelSpinning)
        .finallyDo(interrupted -> { if (interrupted) io.stop(); })
        .andThen(
            this.startEnd(
                () -> {
                  this.setFlywheelVelocity(FlywheelSetpoints.kShootRpm);
                  this.setFeederPower(FeederSetpoints.kFeed);
                },
                () -> { io.stop(); }
            )
        ).withName("Shooting");
  }

  @Override
  public void periodic() {
    // Update hardware/sim inputs
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    // Publish to SmartDashboard
    SmartDashboard.putNumber("Shooter \\ Flywheel \\ Applied Output", inputs.flywheelAppliedOutput);
    SmartDashboard.putNumber("Shooter \\ Flywheel \\ Current", inputs.flywheelCurrent);
    SmartDashboard.putNumber("Shooter \\ Flywheel \\ Target Velocity", flywheelTargetVelocity);
    SmartDashboard.putNumber("Shooter \\ Flywheel \\ Actual Velocity", inputs.flywheelVelocity);

    SmartDashboard.putNumber("Shooter \\ Feeder \\ Applied Output", inputs.feederAppliedOutput);
    SmartDashboard.putNumber("Shooter \\ Feeder \\ Current", inputs.feederCurrent);

    SmartDashboard.putBoolean("Is Flywheel Spinning", isFlywheelSpinning.getAsBoolean());
    SmartDashboard.putBoolean("Is Flywheel Stopped", isFlywheelStopped.getAsBoolean());
  }
}
