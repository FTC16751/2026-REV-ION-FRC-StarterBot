package frc.robot.subsystems.intake;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeSubsystemConstants.ConveyorSetpoints;
import frc.robot.Constants.IntakeSubsystemConstants.IntakeSetpoints;
import frc.robot.Constants.IntakeSubsystemConstants.PivotSetpoints;

/** Intake subsystem that delegates hardware access to an IntakeIO implementation. */
public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIO.IntakeIOInputs inputs = new IntakeIO.IntakeIOInputs();

  public Intake(IntakeIO io) {
    this.io = io;
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
        () -> { this.setIntakePower(IntakeSetpoints.kIntake); this.setConveyorPower(ConveyorSetpoints.kIntake); },
        () -> { this.setIntakePower(0.0); this.setConveyorPower(0.0); }
    ).withName("Intaking");
  }

  public Command runExtakeCommand() {
    return this.startEnd(
        () -> { this.setIntakePower(IntakeSetpoints.kExtake); this.setConveyorPower(ConveyorSetpoints.kExtake); },
        () -> { this.setIntakePower(0.0); this.setConveyorPower(0.0); }
    ).withName("Extaking");
  }

  public Command runIntakeForTime(double seconds) {
    return runIntakeCommand().withTimeout(seconds).withName(String.format("Intaking for %.2fs", seconds));
  }

  public Command runConveyorCommand() {
    return this.startEnd(
        () -> this.setConveyorPower(ConveyorSetpoints.kIntake),
        () -> this.setConveyorPower(0.0)
    ).withName("Run Conveyor");
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

    // Also put a couple values on SmartDashboard for convenience
    SmartDashboard.putNumber("Intake | Pivot | Position (Deg)", inputs.pivotPosition);
  }
}
