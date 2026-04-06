package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.Intake.IntakeSetpoints;
import frc.robot.Constants.Intake.PivotSetpoints;

/**
 * Intake subsystem that delegates hardware access to an IntakeIO
 * implementation.
 */
public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private final Alert intakeDisconnectedAlert =
      new Alert("Intake roller motor disconnected.", AlertType.kError);
  private final Alert pivotDisconnectedAlert =
      new Alert("Intake pivot motor disconnected.", AlertType.kError);

  private double pivotTarget = PivotSetpoints.kRetractedPosition;
  private boolean m_freeDeployMode = false;
  private LoggedMechanism2d mechanism = new LoggedMechanism2d(3, 3);
  private LoggedMechanismLigament2d armLig;

  public Intake(IntakeIO io) {
    this.io = io;
    armLig = new LoggedMechanismLigament2d("IntakeArm", Meters.of(Constants.Intake.ARM_LENGTH_METERS),
        Degrees.of(0.0));
    mechanism.getRoot("IntakeBase", Inches.of(20).in(Meters), Inches.of(0).in(Meters)).append(armLig);
    // Always start retracted — arm retracts on first enable regardless of physical position.
    pivotTarget = PivotSetpoints.kRetractedPosition;
    System.out.println("---> Intake initialized (IO based)");
  }

  /** Convenience constructor: constructs a hardware IO implementation. */
  public Intake() {
    this(new IntakeIOSpark());
  }

  private void setIntakePower(double power) {
    io.setIntakePower(MathUtil.clamp(power, -IntakeSetpoints.kMaxPower, IntakeSetpoints.kMaxPower));
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
        },
        () -> {
          this.setIntakePower(0.0);
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
        },
        () -> {
          this.setIntakePower(0.0);
        }).withName("Extaking");
  }

  public Command runIntakeForTime(double seconds) {
    return runIntakeCommand().withTimeout(seconds).withName(String.format("Intaking for %.2fs", seconds));
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

  public Command agitateCommand() {
    Timer agitateTimer = new Timer();
    return Commands.startRun(
        () -> {
          agitateTimer.restart();
          setPivotPosition(PivotSetpoints.kDeployedPosition);
          setIntakePower(IntakeSetpoints.kIntake);
        },
        () -> {
          setIntakePower(IntakeSetpoints.kIntake);
          if (agitateTimer.advanceIfElapsed(0.3)) {
            if (Math.abs(pivotTarget - PivotSetpoints.kAgitatePosition) < 0.01) {
              setPivotPosition(PivotSetpoints.kDeployedPosition);
            } else {
              setPivotPosition(PivotSetpoints.kAgitatePosition);
            }
          }
        },
        this
    ).finallyDo(() -> setIntakePower(0.0)).withName("Agitate");
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

    // Free-deploy mode: once arm reaches deployed target, release PID entirely.
    // Arm floats freely — gravity holds it down, balls can push up without jamming.
    boolean isDeployTarget = Math.abs(pivotTarget - PivotSetpoints.kDeployedPosition) < 1e-9;
    if (isDeployTarget
        && Math.abs(inputs.pivotPosition - PivotSetpoints.kDeployedPosition) < PivotSetpoints.kCompliantHoldTolerance) {
      m_freeDeployMode = true;
    }
    if (!isDeployTarget) {
      m_freeDeployMode = false; // nudge or retract commanded — PID takes over immediately
    }

    if (m_freeDeployMode) {
      io.setPivotDutyCycle(0.0); // zero output — arm moves freely
    } else {
      io.setPivotPosition(pivotTarget);
    }
    Logger.recordOutput("Intake/freeDeployMode", m_freeDeployMode);
    Logger.recordOutput("Intake/pivotTarget", pivotTarget);
    
    // Map absolute values back to degrees solely for visualizing on the dashboard
    double startPos = PivotSetpoints.kRetractedPosition;
    double endPos = PivotSetpoints.kDeployedPosition;
    double visualDegrees = ((inputs.pivotPosition - startPos) / (endPos - startPos)) * 90.0;
    armLig.setAngle(Degrees.of(visualDegrees));

    Logger.processInputs("Intake", inputs);
    intakeDisconnectedAlert.set(!inputs.intakeConnected);
    pivotDisconnectedAlert.set(!inputs.pivotConnected);
  }

}
