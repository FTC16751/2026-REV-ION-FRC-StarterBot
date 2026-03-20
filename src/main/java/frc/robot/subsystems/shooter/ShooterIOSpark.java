// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

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
import frc.robot.Configs;
import frc.robot.Constants.ShooterSubsystemConstants;

/** Hardware IO implementation for the shooter using Spark Flex controllers. */
public class ShooterIOSpark implements ShooterIO {
    private final SparkFlex flywheelMotor = new SparkFlex(ShooterSubsystemConstants.kFlywheelMotorCanId,
            MotorType.kBrushless);
    private final SparkClosedLoopController flywheelController = flywheelMotor.getClosedLoopController();
    private final RelativeEncoder flywheelEncoder = flywheelMotor.getEncoder();

    private final SparkFlex flywheelFollowerMotor = new SparkFlex(ShooterSubsystemConstants.kFlywheelFollowerMotorCanId,
            MotorType.kBrushless);
    private final SparkFlex feederMotor = new SparkFlex(ShooterSubsystemConstants.kFeederMotorCanId,
            MotorType.kBrushless);
    private final RelativeEncoder feederEncoder = feederMotor.getEncoder();

    // Connection debouncers
    private final Debouncer flyDebouncer = new Debouncer(.5, Debouncer.DebounceType.kFalling);
    private final Debouncer flyFollowerDebouncer = new Debouncer(.5, Debouncer.DebounceType.kFalling);
    private final Debouncer feederDebouncer = new Debouncer(.5, Debouncer.DebounceType.kFalling);
    private SparkClosedLoopController flywheelCLC = flywheelMotor.getClosedLoopController();

    public ShooterIOSpark() {
        flywheelMotor.configure(
                Configs.ShooterSubsystem.flywheelConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        flywheelFollowerMotor.configure(
                Configs.ShooterSubsystem.flywheelFollowerConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        feederMotor.configure(
                Configs.ShooterSubsystem.feederConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // Zero encoder on init (match previous behavior)
        flywheelEncoder.setPosition(0);
    }

    @Override
    public void updateInputs(ShooterIO.ShooterIOInputs inputs) {
        // Flywheel
        sparkStickyFault = false;
        ifOk(flywheelMotor, flywheelEncoder::getVelocity, (value) -> inputs.flywheelVelocity = value);
        ifOk(flywheelMotor, ()-> flywheelCLC.getSetpoint(), (value)-> inputs.flywheelTargetVelocity = value);
        ifOk(flywheelMotor, new DoubleSupplier[] {
                flywheelMotor::getAppliedOutput, flywheelMotor::getBusVoltage
        }, (values) -> {
            inputs.flywheelAppliedVoltage = values[0] * values[1];
        });
        ifOk(flywheelMotor, flywheelMotor::getOutputCurrent, (value) -> inputs.flywheelCurrent = value);
        inputs.flywheelConnected = flyDebouncer.calculate(!sparkStickyFault);
        
        sparkStickyFault = false;
        ifOk(flywheelFollowerMotor, new DoubleSupplier[] {
                flywheelFollowerMotor::getAppliedOutput, flywheelFollowerMotor::getBusVoltage
        }, (values) -> {
            inputs.flywheelAppliedVoltage = values[0] * values[1];
        });
        inputs.flyFollowerConnected = flyFollowerDebouncer.calculate(!sparkStickyFault);

        // Feeder
        
        sparkStickyFault = false;
        ifOk(feederMotor, feederEncoder::getVelocity, (value) -> inputs.feederVelocity = value);
        ifOk(feederMotor, new DoubleSupplier[] {
                feederMotor::getAppliedOutput, feederMotor::getBusVoltage
        }, (values) -> {
            inputs.feederAppliedVoltage = values[0] * values[1];
        });
        ifOk(feederMotor, feederMotor::getOutputCurrent, (value) -> inputs.feederCurrent = value);
        inputs.feederConnected = feederDebouncer.calculate(!sparkStickyFault);
    }

    @Override
    public void setFlywheelVelocity(double rpm) {
        flywheelController.setSetpoint(rpm, ControlType.kVelocity);
    }

    @Override
    public void setFeederPower(double power) {
        feederMotor.set(power);
    }

    @Override
    public void stop() {
        flywheelMotor.stopMotor();
        flywheelFollowerMotor.stopMotor();
        feederMotor.stopMotor();
    }
}
