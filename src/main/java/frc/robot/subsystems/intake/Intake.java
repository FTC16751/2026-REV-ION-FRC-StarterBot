package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeSubsystemConstants.ConveyorSetpoints;
import frc.robot.Constants.IntakeSubsystemConstants.IntakeSetpoints;
import frc.robot.Constants.IntakeSubsystemConstants.PivotSetpoints;

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
    armLig = new LoggedMechanismLigament2d("IntakeArm", Meters.of(Constants.IntakeSubsystemConstants.ARM_LENGTH_METERS),
        Degrees.of(Constants.IntakeSubsystemConstants.PivotSetpoints.kRetractedDegrees));
    mechanism.getRoot("IntakeBase", Inches.of(20).in(Meters), Inches.of(0).in(Meters)).append(armLig);
    System.out.println("---> Intake initialized (IO based)");
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

  private void setPivotPosition(double degrees) {
    io.setPivotPosition(degrees);
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

  public Command deployIntakeCommand() {
    return this.runOnce(() -> setPivotPosition(PivotSetpoints.kDeployedDegrees)).withName("Deploy Intake");
  }

  public Command retractIntakeCommand() {
    return this.runOnce(() -> setPivotPosition(PivotSetpoints.kRetractedDegrees)).withName("Retract Intake");
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    armLig.setAngle(Degrees.of(inputs.pivotPosition));
    Logger.processInputs("Intake", inputs);
  }

}
