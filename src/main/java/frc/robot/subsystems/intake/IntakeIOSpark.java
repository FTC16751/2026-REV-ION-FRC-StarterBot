package frc.robot.subsystems.intake;

import static frc.robot.util.SparkUtil.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.filter.Debouncer;
import frc.robot.Configs;
import frc.robot.Constants.Intake;

/** Hardware IO implementation for the intake using Spark Flex controllers. */
public class IntakeIOSpark implements IntakeIO {
  private final SparkFlex intakeMotor = new SparkFlex(Intake.kIntakeMotorCanId, MotorType.kBrushless);
  private final SparkFlex intakeFollowerMotor = new SparkFlex(Intake.kIntakeFollowerMotorCanId, MotorType.kBrushless);

  private final SparkFlex pivotMotor = new SparkFlex(Intake.kPivotMotorCanId, MotorType.kBrushless);
  private final SparkClosedLoopController pivotController = pivotMotor.getClosedLoopController();
  private final AbsoluteEncoder pivotEncoder = pivotMotor.getAbsoluteEncoder();

  private final Debouncer intakeDebouncer = new Debouncer(.5, Debouncer.DebounceType.kFalling);
  private final Debouncer pivotDebouncer = new Debouncer(.5, Debouncer.DebounceType.kFalling);

  public IntakeIOSpark() {
    intakeMotor.configure(
        Configs.IntakeSubsystem.intakeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    intakeFollowerMotor.configure(
        Configs.IntakeSubsystem.intakeFollowerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    pivotMotor.configure(
        Configs.IntakeSubsystem.pivotConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(IntakeIO.IntakeIOInputs inputs) {
    // Intake
    sparkStickyFault = false;
    ifOk(intakeMotor, new DoubleSupplier[] { intakeMotor::getAppliedOutput, intakeMotor::getBusVoltage }, (values) -> {
      inputs.intakeAppliedVoltage = values[0] * values[1];
    });
    inputs.intakeConnected = intakeDebouncer.calculate(!sparkStickyFault);

    // Pivot
    sparkStickyFault = false;
    ifOk(pivotMotor, pivotEncoder::getPosition, (value) -> inputs.pivotPosition = value);
    ifOk(pivotMotor, pivotEncoder::getVelocity, (value) -> inputs.pivotVelocity = value);
    ifOk(pivotMotor, new DoubleSupplier[] { pivotMotor::getAppliedOutput, pivotMotor::getBusVoltage }, (values) -> {
      inputs.pivotAppliedVoltage = values[0] * values[1];
    });
    ifOk(pivotMotor, pivotMotor::getOutputCurrent, (value) -> inputs.pivotCurrent = value);
    inputs.pivotTargetPosition = pivotController.getSetpoint();
    inputs.pivotConnected = pivotDebouncer.calculate(!sparkStickyFault);
  }

  @Override
  public void setIntakePower(double power) {
    intakeMotor.set(power);
  }

  @Override
  public void setPivotPosition(double position) {
    pivotController.setSetpoint(position, ControlType.kPosition); // motor controller can compute based on kCos itself with feedforward
  }

  @Override
  public void setPivotDutyCycle(double dutyCycle) {
    pivotMotor.set(dutyCycle);
  }

  @Override
  public void stop() {
      intakeMotor.stopMotor();
      pivotMotor.stopMotor();
  }
}
