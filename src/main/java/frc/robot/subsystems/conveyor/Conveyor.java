package frc.robot.subsystems.conveyor;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Conveyor extends SubsystemBase {
  private final ConveyorIO io;
  private final ConveyorIOInputsAutoLogged inputs = new ConveyorIOInputsAutoLogged();

  private final Alert conveyorDisconnectedAlert =
      new Alert("Conveyor motor disconnected.", AlertType.kError);

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
        () -> setConveyorPower(Constants.Conveyor.ConveyorSetpoints.kConveyorSpeedLaunch),
        () -> setConveyorPower(0.0)).withName("Run Conveyor");
  }

  public Command runConveyorIntakeCommand() {
    return this.startEnd(
        () -> setConveyorPower(Constants.Conveyor.ConveyorSetpoints.kConveyorSpeedIntake),
        () -> setConveyorPower(0.0)).withName("Run Conveyor (Intake)");
  }

  public Command runConveyorReverseCommand() {
    return this.startEnd(
        () -> setConveyorPower(Constants.Conveyor.ConveyorSetpoints.kConveyorExtakeSpeed),
        () -> setConveyorPower(0.0)).withName("Reverse Conveyor");
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Conveyor", inputs);
  }
}