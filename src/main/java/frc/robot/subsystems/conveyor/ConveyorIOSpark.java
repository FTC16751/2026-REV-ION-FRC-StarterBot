package frc.robot.subsystems.conveyor;

import static frc.robot.util.SparkUtil.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.filter.Debouncer;
import frc.robot.Configs;
import frc.robot.Constants.Conveyor;

/** Hardware IO implementation for the conveyor using Spark Flex controllers. */
public class ConveyorIOSpark implements ConveyorIO {
  private final SparkFlex conveyorMotor = new SparkFlex(Conveyor.kConveyorMotorCanId, MotorType.kBrushless);
  private final Debouncer conveyorDebouncer = new Debouncer(.5, Debouncer.DebounceType.kFalling);

  public ConveyorIOSpark() {
    conveyorMotor.configure(
        Configs.ConveyorSubsystem.conveyorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ConveyorIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(conveyorMotor, new DoubleSupplier[] { conveyorMotor::getAppliedOutput, conveyorMotor::getBusVoltage },
        (values) -> inputs.conveyorAppliedVoltage = values[0] * values[1]);
    ifOk(conveyorMotor, conveyorMotor::getOutputCurrent, (value) -> inputs.conveyorCurrent = value);
    inputs.conveyorConnected = conveyorDebouncer.calculate(!sparkStickyFault);
  }

  @Override
  public void setConveyorPower(double power) {
    conveyorMotor.set(power);
  }
}