package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.util.SparkUtil.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.Intake;

/** Hardware IO implementation for the intake using Spark Flex controllers. */
public class IntakeIOSpark implements IntakeIO {
  private final SparkFlex intakeMotor = new SparkFlex(Intake.kIntakeMotorCanId, MotorType.kBrushless);
  private final SparkFlex conveyorMotor = new SparkFlex(Intake.kConveyorMotorCanId, MotorType.kBrushless);

  private final SparkFlex pivotMotor = new SparkFlex(Intake.kPivotMotorCanId, MotorType.kBrushless);
  private final SparkClosedLoopController pivotController = pivotMotor.getClosedLoopController();
  private final RelativeEncoder pivotEncoder = pivotMotor.getEncoder();

  private final Debouncer intakeDebouncer = new Debouncer(.5, Debouncer.DebounceType.kFalling);
  private final Debouncer conveyorDebouncer = new Debouncer(.5, Debouncer.DebounceType.kFalling);
  private final Debouncer pivotDebouncer = new Debouncer(.5, Debouncer.DebounceType.kFalling);

  public IntakeIOSpark() {
    intakeMotor.configure(
        Configs.IntakeSubsystem.intakeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    conveyorMotor.configure(
        Configs.IntakeSubsystem.conveyorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    pivotMotor.configure(
        Configs.IntakeSubsystem.pivotConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    pivotEncoder.setPosition(0.0);
  }

  @Override
  public void updateInputs(IntakeIO.IntakeIOInputs inputs) {
    // Intake
    sparkStickyFault = false;
    ifOk(intakeMotor, new DoubleSupplier[] { intakeMotor::getAppliedOutput, intakeMotor::getBusVoltage }, (values) -> {
      inputs.intakeAppliedVoltage = values[0] * values[1];
    });
    inputs.intakeConnected = intakeDebouncer.calculate(!sparkStickyFault);

    // Conveyor
    sparkStickyFault = false;
    ifOk(conveyorMotor, new DoubleSupplier[] { conveyorMotor::getAppliedOutput, conveyorMotor::getBusVoltage },
        (values) -> {
          inputs.conveyorAppliedVoltage = values[0] * values[1];
        });
    inputs.conveyorConnected = conveyorDebouncer.calculate(!sparkStickyFault);

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
  public void setIntakeSpeed(AngularVelocity speed) {
    intakeMotor.getClosedLoopController().setSetpoint(speed.in(RPM), ControlType.kVelocity);
  }

  @Override
  public void setConveyorPower(double power) {
    conveyorMotor.set(power);
  }

  @Override
  public void setPivotPosition(double degrees) {
    pivotController.setSetpoint(degrees, ControlType.kPosition);
  }

  @Override
  public void zeroPivotPosition(boolean bottom) {
    pivotMotor.getEncoder().setPosition(bottom ? Constants.Intake.PivotSetpoints.kDeployedDegrees : Constants.Intake.PivotSetpoints.kRetractedDegrees);
  }

  @Override
  public void stop() {
      intakeMotor.stopMotor();
      conveyorMotor.stopMotor();
      pivotMotor.stopMotor();
  }
}
