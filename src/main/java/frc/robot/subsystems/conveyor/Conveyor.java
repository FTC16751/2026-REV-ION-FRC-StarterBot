package frc.robot.subsystems.conveyor;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Conveyor extends SubsystemBase {
  private final ConveyorIO io;
  private final ConveyorIOInputsAutoLogged inputs = new ConveyorIOInputsAutoLogged();

  public Conveyor(ConveyorIO io) {
    this.io = io;
    System.out.println("---> Conveyor initialized");
  }

  public Conveyor() {
    this(new ConveyorIOSpark());
  }

  public void setConveyorPower(double power) {
    io.setConveyorPower(MathUtil.clamp(power, -1.0, 1.0));
  }

  public Command runConveyorCommand() {
    return this.startEnd(
        () -> setConveyorPower(Constants.Conveyor.ConveyorSetpoints.kIntake),
        () -> setConveyorPower(0.0)).withName("Run Conveyor");
  }

  public Command runConveyorReverseCommand() {
    return this.startEnd(
        () -> setConveyorPower(Constants.Conveyor.ConveyorSetpoints.kExtake),
        () -> setConveyorPower(0.0)).withName("Reverse Conveyor");
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Conveyor", inputs);
  }
}