package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
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
  private LoggedMechanism2d mechanism = new LoggedMechanism2d(3, 3);
  private LoggedMechanismLigament2d armLig;

  public Intake(IntakeIO io) {
    this.io = io;
    armLig = new LoggedMechanismLigament2d("IntakeArm", Meters.of(Constants.Intake.ARM_LENGTH_METERS),
        Degrees.of(Constants.Intake.PivotSetpoints.kRetractedDegrees));
    mechanism.getRoot("IntakeBase", Inches.of(20).in(Meters), Inches.of(0).in(Meters)).append(armLig);
    System.out.println("---> Intake initialized (IO based)");
    slamBottom().debounce(1).onTrue(runOnce(this::zeroPivotPosition).withName("Slammed into Bottom Stop"));
    slamTop().debounce(1).onTrue(runOnce(this::maxPivotPosition).withName("Slammed into Top Stop"));
  
  }

  /** Convenience constructor: constructs a hardware IO implementation. */
  public Intake() {
    this(new IntakeIOSpark());
  }

  @SuppressWarnings("unused")
  @Deprecated
  private void setIntakePower(double power) {
    io.setIntakePower(MathUtil.clamp(power, -1.0, 1.0));
  }
  private void setIntakeSpeed(AngularVelocity speed) {
    io.setIntakeSpeed(speed);
  }

  private void setConveyorPower(double power) {
    io.setConveyorPower(MathUtil.clamp(power, -1.0, 1.0));
  }

  private void setPivotPosition(double degrees) {
    io.setPivotPosition(degrees);
  }

  private void zeroPivotPosition() {
    io.zeroPivotPosition(true);
  }
  private void maxPivotPosition() {
    io.zeroPivotPosition(false);
  }

  public Command runIntakeCommand() {
    return this.startEnd(
        () -> {
          this.setIntakeSpeed(IntakeSetpoints.kIntakeSpeed);
          this.setConveyorPower(ConveyorSetpoints.kIntake);
        },
        () -> {
          this.setIntakeSpeed(RPM.zero());
          this.setConveyorPower(0.0);
        }).withName("Intaking");
  }

  public Command runIntakeOnlyCommand() {
    return this.startEnd(
        () -> this.setIntakePower(IntakeSetpoints.kIntake),
        () -> this.setIntakePower(0.0)
    ).withName("Intaking (no conveyor)");
  }

  public Command runExtakeCommand() {
    return this.startEnd(
        () -> {
          this.setIntakeSpeed(IntakeSetpoints.kExtakeSpeed);
          this.setConveyorPower(ConveyorSetpoints.kExtake);
        },
        () -> {
          this.setIntakeSpeed(RPM.zero());
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

  public Command deployIntakeCommand() {
    return this.runOnce(() -> setPivotPosition(PivotSetpoints.kDeployedDegrees)).withName("Deploy Intake");
  }

  public Command retractIntakeCommand() {
    return this.runOnce(() -> setPivotPosition(PivotSetpoints.kRetractedDegrees)).withName("Retract Intake");
  }

  public Command partialIntakeCommand(DoubleSupplier percent) {
    Interpolator<Double> interpolator = Interpolator.forDouble();
    double angle = interpolator.interpolate(PivotSetpoints.kRetractedDegrees, PivotSetpoints.kDeployedDegrees, percent.getAsDouble());
    return this.runOnce(() -> setPivotPosition(angle)).withName("Deploy Intake");
  }

  BooleanSupplier excessCurrent = () -> Math.abs( inputs.pivotCurrent) > 20; // should not take more than half the current limit.
  BooleanSupplier zeroVelocity = () -> inputs.pivotVelocity < 1e-4; // low velocity means it's stopped  
    BooleanSupplier negativeOutput = () -> inputs.pivotAppliedVoltage < 0; // negative applied voltage means it's trying to move towards deployed;
    BooleanSupplier positiveOutput = () -> inputs.pivotAppliedVoltage > 0; // negative applied voltage means it's trying to move towards retracted;
  public Trigger slamBottom() {
    return new Trigger(excessCurrent).and(negativeOutput).and(zeroVelocity);
  }
  public Trigger slamTop() {
    return new Trigger(excessCurrent).and(positiveOutput).and(zeroVelocity);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    armLig.setAngle(Degrees.of(inputs.pivotPosition));
    Logger.processInputs("Intake", inputs);
  }

}
