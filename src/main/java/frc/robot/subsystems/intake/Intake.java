package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.Intake.ConveyorSetpoints;
import frc.robot.Constants.Intake.IntakeSetpoints;
import frc.robot.Constants.Intake.PivotSetpoints;

/**
 * Intake subsystem that delegates hardware access to an IntakeIO
 * implementation.
 */
public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private double pivotTarget = PivotSetpoints.kRetractedPosition;
  private boolean inCompliantHold = false;
  private final Debouncer pivotStallDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);
  private LoggedMechanism2d mechanism = new LoggedMechanism2d(3, 3);
  private LoggedMechanismLigament2d armLig;

  public Intake(IntakeIO io) {
    this.io = io;
    armLig = new LoggedMechanismLigament2d("IntakeArm", Meters.of(Constants.Intake.ARM_LENGTH_METERS),
        Degrees.of(0.0));
    mechanism.getRoot("IntakeBase", Inches.of(20).in(Meters), Inches.of(0).in(Meters)).append(armLig);
    // Initialize pivotTarget from actual encoder position so the arm holds in place on enable
    // rather than snapping to kRetractedPosition if the arm is somewhere else.
    IntakeIO.IntakeIOInputs initialInputs = new IntakeIO.IntakeIOInputs();
    io.updateInputs(initialInputs);
    pivotTarget = MathUtil.clamp(initialInputs.pivotPosition,
        Math.min(PivotSetpoints.kRetractedPosition, PivotSetpoints.kDeployedPosition),
        Math.max(PivotSetpoints.kRetractedPosition, PivotSetpoints.kDeployedPosition));
    System.out.println("---> Intake initialized (IO based)");
    slamBottom().debounce(1).onTrue(runOnce(this::zeroPivotPosition));
  }

  /** Convenience constructor: constructs a hardware IO implementation. */
  public Intake() {
    this(new IntakeIOSpark());
  }

  private void setIntakePower(double power) {
    io.setIntakePower(MathUtil.clamp(power, -1.0, 1.0));
  }

  private void setConveyorPower(double power) {
    io.setConveyorPower(MathUtil.clamp(power, -1.0, 1.0));
  }

  private void setPivotPosition(double position) {
    pivotTarget = MathUtil.clamp(position,
        Math.min(PivotSetpoints.kRetractedPosition, PivotSetpoints.kDeployedPosition),
        Math.max(PivotSetpoints.kRetractedPosition, PivotSetpoints.kDeployedPosition));
  }

  private void zeroPivotPosition() {
    io.zeroPivotPosition();
  }

  public Command runIntakeCommand() {
    return this.startEnd(
        () -> {
          this.setIntakePower(IntakeSetpoints.kIntake);
          this.setConveyorPower(ConveyorSetpoints.kIntake);
        },
        () -> {
          this.setIntakePower(0.0);
          this.setConveyorPower(0.0);
        }).withName("Intaking");
  }

  public Command runIntakeOnlyCommand(DoubleSupplier power) {
    return this.run(() -> this.setIntakePower(power.getAsDouble()))
        .finallyDo(() -> this.setIntakePower(0.0))
        .withName("Intaking (no conveyor)");
  }

  public Command runExtakeCommand() {
    return this.startEnd(
        () -> {
          this.setIntakePower(IntakeSetpoints.kExtake);
          this.setConveyorPower(ConveyorSetpoints.kExtake);
        },
        () -> {
          this.setIntakePower(0.0);
          this.setConveyorPower(0.0);
        }).withName("Extaking");
  }

  public Command runIntakeForTime(double seconds) {
    return runIntakeCommand().withTimeout(seconds).withName(String.format("Intaking for %.2fs", seconds));
  }

  public Command runConveyorCommand() {
    return this.startEnd(
        () -> this.setConveyorPower(ConveyorSetpoints.kIntake),
        () -> this.setConveyorPower(0.0)).withName("Run Conveyor");
  }

  public Command runConveyorReverseCommand() {
    return this.startEnd(
        () -> this.setConveyorPower(ConveyorSetpoints.kExtake),
        () -> this.setConveyorPower(0.0)).withName("Reverse Conveyor");
  }

  /**
   * Nudges the pivot in either direction based on a joystick axis.
   * Positive axis moves toward deployed, negative toward retracted.
   * No subsystem requirement so it runs alongside deploy/retract presets.
   */
  public Command nudgePivotCommand(DoubleSupplier axisSupplier) {
    return Commands.run(() -> {
      double axis = axisSupplier.getAsDouble();
      // deployed < retracted in encoder units, so positive axis → subtract to go toward deployed
      setPivotPosition(pivotTarget - axis * PivotSetpoints.kNudgePerCycle);
    }).withName("Nudge Pivot");
  }

  public void toggleDeployRetract() {
    if (Math.abs(pivotTarget - PivotSetpoints.kDeployedPosition) < Math.abs(pivotTarget - PivotSetpoints.kRetractedPosition)) {
      setPivotPosition(PivotSetpoints.kRetractedPosition);
    } else {
      setPivotPosition(PivotSetpoints.kDeployedPosition);
    }
  }

  /** Nudge pivot toward retracted (up) while held; arm holds position on release. No subsystem requirement so it runs alongside other intake commands. */
  public Command nudgePivotUpCommand() {
    return Commands.run(() -> setPivotPosition(pivotTarget + PivotSetpoints.kNudgePerCycle))
        .withName("Nudge Pivot Up");
  }

  public Command deployIntakeCommand() {
    return this.runOnce(() -> setPivotPosition(PivotSetpoints.kDeployedPosition)).withName("Deploy Intake");
  }

  public Command retractIntakeCommand() {
    return this.runOnce(() -> setPivotPosition(PivotSetpoints.kRetractedPosition)).withName("Retract Intake");
  }

  public Command partialIntakeCommand(DoubleSupplier percent) {
    Interpolator<Double> interpolator = Interpolator.forDouble();
    double angle = interpolator.interpolate(PivotSetpoints.kRetractedPosition, PivotSetpoints.kDeployedPosition, percent.getAsDouble());
    return this.runOnce(() -> setPivotPosition(angle)).withName("Deploy Intake");
  }

  public Trigger slamBottom() {
    return new Trigger(() -> (inputs.pivotCurrent > 20) && (Math.abs(inputs.pivotVelocity) < 0.05));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    // Stall protection: if motor is fighting but not moving, snap target to actual position
    boolean stalling = inputs.pivotCurrent > PivotSetpoints.kStallCurrentAmps
        && Math.abs(inputs.pivotVelocity) < 0.05
        && Math.abs(inputs.pivotPosition - pivotTarget) > PivotSetpoints.kPositionTolerance;
    boolean isStalled = pivotStallDebouncer.calculate(stalling);

    // Compliant hold latch: enter when arm reaches deployed target, stay until target changes away.
    // This prevents the PID from fighting a jammed ball even when it pushes the arm beyond tolerance.
    boolean isDeployTarget = Math.abs(pivotTarget - PivotSetpoints.kDeployedPosition) < 1e-9;
    if (isDeployTarget) {
      if (Math.abs(inputs.pivotPosition - pivotTarget) < PivotSetpoints.kCompliantHoldTolerance || isStalled) {
        inCompliantHold = true; // latch on
      }
    } else {
      inCompliantHold = false; // target changed (e.g. retract commanded) — exit compliant mode
    }

    // Only snap the target to the stall position if we haven't safely yielded into compliant mode
    if (isStalled && !(PivotSetpoints.kUseCompliantHold && inCompliantHold)) {
      pivotTarget = MathUtil.clamp(inputs.pivotPosition,
          Math.min(PivotSetpoints.kRetractedPosition, PivotSetpoints.kDeployedPosition),
          Math.max(PivotSetpoints.kRetractedPosition, PivotSetpoints.kDeployedPosition));
    }

    if (PivotSetpoints.kUseCompliantHold && inCompliantHold) {
      double t = (inputs.pivotPosition - PivotSetpoints.kRetractedPosition)
          / (PivotSetpoints.kDeployedPosition - PivotSetpoints.kRetractedPosition);
      double angle = (1.0 - t) * (Math.PI / 2.0);
      io.setPivotDutyCycle(PivotSetpoints.kCos * Math.cos(angle));
    } else {
      io.setPivotPosition(pivotTarget);
    }
    Logger.recordOutput("Intake/inCompliantHold", inCompliantHold);
    Logger.recordOutput("Intake/pivotTarget", pivotTarget);
    
    // Map absolute values back to degrees solely for visualizing on the dashboard
    double startPos = PivotSetpoints.kRetractedPosition;
    double endPos = PivotSetpoints.kDeployedPosition;
    double visualDegrees = ((inputs.pivotPosition - startPos) / (endPos - startPos)) * 90.0;
    armLig.setAngle(Degrees.of(visualDegrees));

    Logger.processInputs("Intake", inputs);
  }

}
