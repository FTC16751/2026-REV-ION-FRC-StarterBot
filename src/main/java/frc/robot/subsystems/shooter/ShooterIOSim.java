// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

import frc.robot.Constants.ShooterSubsystemConstants;

/**
 * Physics-based simulation IO for the shooter using DCMotorSim +
 * LinearSystemId.createFlywheelSystem.
 */
public class ShooterIOSim implements ShooterIO {
        // Two Neo Vortex motors
        private static final DCMotor FLYWHEEL_GEARBOX = DCMotor.getNeoVortex(2);

        // Create SparkFlex objects matching real hardware (used by SparkFlexSim)
        private final SparkFlex flywheelSpark = new SparkFlex(ShooterSubsystemConstants.kFlywheelMotorCanId,
                        MotorType.kBrushless);
        private final SparkFlex flywheelFollowerSpark = new SparkFlex(
                        ShooterSubsystemConstants.kFlywheelFollowerMotorCanId,
                        MotorType.kBrushless);
        private final SparkFlex feederSpark = new SparkFlex(ShooterSubsystemConstants.kFeederMotorCanId,
                        MotorType.kBrushless);

        // SparkFlexSim instances link the simulated physics to the controller objects.
        private final SparkFlexSim flywheelSparkSim = new SparkFlexSim(flywheelSpark, FLYWHEEL_GEARBOX);
        private final SparkFlexSim flywheelFollowerSparkSim = new SparkFlexSim(flywheelFollowerSpark, FLYWHEEL_GEARBOX);
        // Feeder uses a single Neo Vortex motor for simulation
        private static final DCMotor FEEDER_GEARBOX = DCMotor.getNeoVortex(1);
        private final SparkFlexSim feederSparkSim = new SparkFlexSim(feederSpark, FEEDER_GEARBOX);
        private SparkClosedLoopController flywheelCLC = flywheelSpark.getClosedLoopController();

        // Flywheel physics model (authoritative physics for the flywheel)
        private static final double MOMENT_OF_INERTIA = 0.009; // kg*m^2 (given)
        private static final double GEARING = 1.0; // 1:1
        FlywheelSim flywheelSim = new FlywheelSim(
                        LinearSystemId.createFlywheelSystem(FLYWHEEL_GEARBOX, MOMENT_OF_INERTIA, GEARING),
                        FLYWHEEL_GEARBOX);
        private double lastTime = Timer.getFPGATimestamp();

        public ShooterIOSim() {
                flywheelSpark.configure(
                                frc.robot.Configs.ShooterSubsystem.flywheelConfig,
                                ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);
                flywheelFollowerSpark.configure(
                                frc.robot.Configs.ShooterSubsystem.flywheelFollowerConfig,
                                ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);
                feederSpark.configure(
                                frc.robot.Configs.ShooterSubsystem.feederConfig,
                                ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);
        }

        @Override
        public void updateInputs(ShooterIO.ShooterIOInputs inputs) {
                double now = Timer.getFPGATimestamp();
                double dt = Math.max(0.0, now - lastTime);
                lastTime = now;
                // Use the RoboRio input voltage for sim iteration (allows battery simulation)
                double iterateBattery = RoboRioSim.getVInVoltage();

                // Drive authoritative flywheel physics using the applied output from the
                // controller (from the previous timestep) scaled by the current sim battery.
                double appliedVolts = flywheelSpark.getAppliedOutput() * iterateBattery;
                flywheelSim.setInputVoltage(appliedVolts);
                flywheelSim.update(dt);
                double rotorRPM = flywheelSim.getAngularVelocityRPM();

                // Push the simulated encoder state into the Spark sim so controllers read
                // the current plant state.
                flywheelSparkSim.getRelativeEncoderSim().setVelocity(rotorRPM);

                // Now iterate the Spark sims so their controllers read the encoder state we
                // just wrote and compute outputs for this timestep. Pass the rotor
                // velocity to the Spark sim iterate call so it can use it internally.
                flywheelSparkSim.iterate(rotorRPM, iterateBattery, dt);
                flywheelFollowerSparkSim.iterate(rotorRPM, iterateBattery, dt);
                feederSparkSim.iterate(0.0, iterateBattery, dt);

                // Read applied current from sims and update battery loaded voltage
                RoboRioSim.setVInVoltage(
                                BatterySim.calculateDefaultBatteryLoadedVoltage(flywheelSim.getCurrentDrawAmps()));

                // Fill inputs from physics (use rotor state from flywheelSim and applied
                // voltage computed earlier)
                inputs.flywheelVelocity = rotorRPM;
                inputs.flywheelAppliedVoltage = appliedVolts;
                inputs.flywheelTargetVelocity = flywheelCLC.getSetpoint();
                inputs.flywheelCurrent = Math.abs(flywheelSim.getCurrentDrawAmps());
                inputs.flyFollowerAppliedVoltage = flywheelSpark.getAppliedOutput() * iterateBattery;

                inputs.feederAppliedVoltage = feederSpark.getAppliedOutput() * iterateBattery;
                inputs.feederCurrent = feederSpark.getOutputCurrent();
        }

        @Override
        public void setFlywheelVelocity(double rpm) {
                // Forward the setpoint to the SparkFlex closed-loop controller so SparkFlexSim
                // can simulate it.
                flywheelSpark.getClosedLoopController().setSetpoint(rpm,
                                com.revrobotics.spark.SparkBase.ControlType.kMAXMotionVelocityControl);
        }

        @Override
        public void setFeederPower(double power) {
                MathUtil.clamp(power, -1.0, 1.0);
        }

        @Override
        public void stop() {
                flywheelSpark.getClosedLoopController().setSetpoint(0.0,
                                com.revrobotics.spark.SparkBase.ControlType.kMAXMotionVelocityControl);
                flywheelSpark.stopMotor();
                flywheelFollowerSpark.stopMotor();
                feederSpark.stopMotor();
        }
}
