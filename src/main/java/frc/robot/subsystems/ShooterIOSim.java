// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.sim.SparkFlexSim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ShooterSubsystemConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;

/** Physics-based simulation IO for the shooter using DCMotorSim + LinearSystemId.createFlywheelSystem. */
public class ShooterIOSim implements ShooterIO {
  // Physical configuration
  private static final double MOMENT_OF_INERTIA = 0.009; // kg*m^2 (given)
  private static final double GEARING = 1.0; // 1:1
  // Two Neo Vortex motors
  private static final DCMotor FLYWHEEL_GEARBOX = DCMotor.getNeoVortex(2);
  // Spark Flex controller sims for leader, follower, and feeder
  private final SparkFlexSim flywheelControllerSim;
  private final SparkFlexSim flywheelFollowerControllerSim;
  private final SparkFlexSim feederControllerSim;

  // Simulation objects
  private final DCMotorSim flywheelSim;

  // Controller for commanding voltage to reach a target velocity (simple P controller)
  private double targetFlywheelVelocityRpm = 0.0;
  private double appliedVolts = 0.0;
  private static final double VELOCITY_P_GAIN = 0.0025; // V per RPM error (tuneable)

  // Feeder sim
  private double simulatedFeederOutput = 0.0;

  private double lastTime = Timer.getFPGATimestamp();

  public ShooterIOSim() {
  // Create a linear model of the flywheel and DCMotorSim
  // Note: use createDCMotorSystem with the flywheel's moment of inertia and gearing
  // which provides a compatible linear system for DCMotorSim in this WPILib version.
  flywheelSim = new DCMotorSim(
    LinearSystemId.createDCMotorSystem(FLYWHEEL_GEARBOX, MOMENT_OF_INERTIA, GEARING),
    FLYWHEEL_GEARBOX);

  // Create SparkFlexSim instances for the real controller CAN IDs (use default ctor and rely on the
  // sim library to register them to the controllers internally)
  flywheelControllerSim = new SparkFlexSim(ShooterSubsystemConstants.kFlywheelMotorCanId, MotorType.kBrushless);
  flywheelFollowerControllerSim = new SparkFlexSim(ShooterSubsystemConstants.kFlywheelFollowerMotorCanId, MotorType.kBrushless);
  feederControllerSim = new SparkFlexSim(ShooterSubsystemConstants.kFeederMotorCanId, MotorType.kBrushless);
  }

  @Override
  public void updateInputs(ShooterIO.ShooterIOInputs inputs) {
    double now = Timer.getFPGATimestamp();
    double dt = Math.max(0.0, now - lastTime);
    lastTime = now;

    // Compute control voltage via simple P controller on RPM error
    double currentRpm = flywheelSim.getAngularVelocityRadPerSec() * 60.0 / (2.0 * Math.PI);
    double error = targetFlywheelVelocityRpm - currentRpm;
    appliedVolts = MathUtil.clamp(VELOCITY_P_GAIN * error, -12.0, 12.0);

    // Feed voltage into physics sim
    flywheelSim.setInputVoltage(appliedVolts);
    flywheelSim.update(dt);

    // Read simulated sensors
    double rotorRadPerSec = flywheelSim.getAngularVelocityRadPerSec();
    double rotorRpm = rotorRadPerSec * 60.0 / (2.0 * Math.PI);

    inputs.flywheelVelocity = rotorRpm; // RPM
    inputs.flywheelAppliedOutput = appliedVolts; // volts
    inputs.flywheelCurrent = Math.abs(flywheelSim.getCurrentDrawAmps());

    // Feeder simple sim
    inputs.feederAppliedOutput = simulatedFeederOutput;
    inputs.feederCurrent = Math.abs(simulatedFeederOutput) * 5.0; // arbitrary small current
  }

  @Override
  public void setFlywheelVelocity(double rpm) {
    targetFlywheelVelocityRpm = rpm;
  }

  @Override
  public void setFeederPower(double power) {
    simulatedFeederOutput = MathUtil.clamp(power, -1.0, 1.0);
  }

  @Override
  public void stop() {
    targetFlywheelVelocityRpm = 0.0;
    simulatedFeederOutput = 0.0;
    appliedVolts = 0.0;
    flywheelSim.setInputVoltage(0.0);
  }
}
