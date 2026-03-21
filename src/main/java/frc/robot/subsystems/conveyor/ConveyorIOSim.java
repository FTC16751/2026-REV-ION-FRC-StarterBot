package frc.robot.subsystems.conveyor;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Constants.Conveyor;

public class ConveyorIOSim implements ConveyorIO {
  private final SparkFlex conveyorSpark = new SparkFlex(Conveyor.kConveyorMotorCanId, MotorType.kBrushless);
  private final SparkFlexSim conveyorSim = new SparkFlexSim(conveyorSpark, DCMotor.getNeoVortex(1));
  private double lastTime = Timer.getFPGATimestamp();

  @Override
  public void updateInputs(ConveyorIOInputs inputs) {
    double now = Timer.getFPGATimestamp();
    double dt = Math.max(0.0, now - lastTime);
    lastTime = now;

    double battery = RobotController.getBatteryVoltage();
    conveyorSim.iterate(now, dt, battery);

    inputs.conveyorAppliedVoltage = conveyorSpark.getAppliedOutput() * battery;
    // SparkFlexSim does not calculate current without a WPILib physics sim attached
    inputs.conveyorCurrent = 0.0;
  }

  @Override
  public void setConveyorPower(double power) {
    conveyorSpark.set(power);
  }
}