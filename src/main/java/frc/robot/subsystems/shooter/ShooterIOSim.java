// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.RobotController;

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
    private final SparkFlex flywheelFollowerSpark = new SparkFlex(ShooterSubsystemConstants.kFlywheelFollowerMotorCanId,
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
            LinearSystemId.createFlywheelSystem(FLYWHEEL_GEARBOX, MOMENT_OF_INERTIA, GEARING), FLYWHEEL_GEARBOX, null);
    private double lastTime = Timer.getFPGATimestamp();

    public ShooterIOSim() {
        // No additional startup actions required for the SparkFlexSim instances here.
    }

    @Override
    public void updateInputs(ShooterIO.ShooterIOInputs inputs) {
        double now = Timer.getFPGATimestamp();
        double dt = Math.max(0.0, now - lastTime);
        lastTime = now;

        // Let REVLib's SparkFlexSim iterate so controllers update their internal state
        // (compute applied outputs).
        double battery = RobotController.getBatteryVoltage();
        flywheelSparkSim.iterate(now, dt, battery);
        flywheelFollowerSparkSim.iterate(now, dt, battery);
        feederSparkSim.iterate(now, dt, battery);

        // Read the controller's applied fraction and convert to voltage to drive our
        // flywheel physics.
        double appliedVolts = flywheelSpark.getAppliedOutput() * battery;

        // Step the authoritative flywheel physics using the computed voltage
        flywheelSim.setInputVoltage(appliedVolts);
        flywheelSim.update(dt);

        double rotorRadPerSec = flywheelSim.getAngularVelocityRadPerSec();

        // Push simulated encoder state back into the SparkFlex external encoder sim so
        // the controller can read it
        var encSim = flywheelSparkSim.getExternalEncoderSim();
        encSim.setVelocity(rotorRadPerSec);

        // Fill inputs from physics
        double rotorRpm = rotorRadPerSec * 60.0 / (2.0 * Math.PI);
        inputs.flywheelVelocity = rotorRpm;
        inputs.flywheelAppliedVoltage = appliedVolts;
        inputs.flywheelTargetVelocity = flywheelCLC.getSetpoint();
        inputs.flywheelCurrent = Math.abs(flywheelSim.getCurrentDrawAmps());
        inputs.flyFollowerAppliedVoltage = appliedVolts;

        // Feeder (read from SparkFlex if available)
        inputs.feederAppliedVoltage = feederSpark.getAppliedOutput();
        inputs.feederCurrent = feederSpark.getOutputCurrent();
    }

    @Override
    public void setFlywheelVelocity(double rpm) {
        // Forward the setpoint to the SparkFlex closed-loop controller so SparkFlexSim
        // can simulate it.
        try {
            flywheelSpark.getClosedLoopController().setSetpoint(rpm,
                    com.revrobotics.spark.SparkBase.ControlType.kMAXMotionVelocityControl);
        } catch (Exception e) {
            // If closed-loop controller unavailable in this context, ignore and rely on
            // higher-level sim
        }
    }

    @Override
    public void setFeederPower(double power) {
        MathUtil.clamp(power, -1.0, 1.0);
    }

    @Override
    public void stop() {
        try {
            flywheelSpark.getClosedLoopController().setSetpoint(0.0,
                    com.revrobotics.spark.SparkBase.ControlType.kMAXMotionVelocityControl);
        } catch (Exception ignored) {
        }
        try {
            flywheelSpark.stopMotor();
            flywheelFollowerSpark.stopMotor();
            feederSpark.stopMotor();
        } catch (Exception ignored) {
        }
    }
}
