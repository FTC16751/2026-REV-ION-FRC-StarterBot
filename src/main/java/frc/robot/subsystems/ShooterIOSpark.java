// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Configs;
import frc.robot.Constants.ShooterSubsystemConstants;

/** Hardware IO implementation for the shooter using Spark Flex controllers. */
public class ShooterIOSpark implements ShooterIO {
  private final SparkFlex flywheelMotor = new SparkFlex(ShooterSubsystemConstants.kFlywheelMotorCanId, MotorType.kBrushless);
  private final SparkClosedLoopController flywheelController = flywheelMotor.getClosedLoopController();
  private final RelativeEncoder flywheelEncoder = flywheelMotor.getEncoder();

  private final SparkFlex flywheelFollowerMotor = new SparkFlex(ShooterSubsystemConstants.kFlywheelFollowerMotorCanId, MotorType.kBrushless);
  private final SparkFlex feederMotor = new SparkFlex(ShooterSubsystemConstants.kFeederMotorCanId, MotorType.kBrushless);

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
    try {
      inputs.flywheelVelocity = flywheelEncoder.getVelocity();
    } catch (Exception e) {
      inputs.flywheelVelocity = 0.0;
    }
    try {
      inputs.flywheelAppliedOutput = flywheelMotor.getAppliedOutput();
      inputs.flywheelCurrent = flywheelMotor.getOutputCurrent();
    } catch (Exception e) {
      inputs.flywheelAppliedOutput = 0.0;
      inputs.flywheelCurrent = 0.0;
    }

    // Feeder
    try {
      inputs.feederAppliedOutput = feederMotor.getAppliedOutput();
      inputs.feederCurrent = feederMotor.getOutputCurrent();
    } catch (Exception e) {
      inputs.feederAppliedOutput = 0.0;
      inputs.feederCurrent = 0.0;
    }
  }

  @Override
  public void setFlywheelVelocity(double rpm) {
    flywheelController.setSetpoint(rpm, ControlType.kMAXMotionVelocityControl);
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
