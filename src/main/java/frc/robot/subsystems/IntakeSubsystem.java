// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.IntakeSubsystemConstants;
import frc.robot.Constants.IntakeSubsystemConstants.ConveyorSetpoints;
import frc.robot.Constants.IntakeSubsystemConstants.IntakeSetpoints;
import frc.robot.Constants.IntakeSubsystemConstants.PivotSetpoints;

public class IntakeSubsystem extends SubsystemBase {
  // Initialize intake SPARK. We will use open loop control for this.
  private SparkFlex intakeMotor =
      new SparkFlex(IntakeSubsystemConstants.kIntakeMotorCanId, MotorType.kBrushless);

  // Initialize conveyor SPARK. We will use open loop control for this.
  private SparkFlex conveyorMotor =
      new SparkFlex(IntakeSubsystemConstants.kConveyorMotorCanId, MotorType.kBrushless);

  // Initialize pivot SPARK. We use closed loop position control for this.
  private SparkFlex pivotMotor =
      new SparkFlex(IntakeSubsystemConstants.kPivotMotorCanId, MotorType.kBrushless);
  private SparkClosedLoopController pivotController = pivotMotor.getClosedLoopController();
  private RelativeEncoder pivotEncoder = pivotMotor.getEncoder();

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    /*
     * Apply the appropriate configurations to the SPARKs.
     *
     * kResetSafeParameters is used to get the SPARK to a known state. This
     * is useful in case the SPARK is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
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

    // Assume the robot starts with the intake retracted (0 degrees)
    pivotEncoder.setPosition(0.0);

    System.out.println("---> IntakeSubsystem initialized");
  }

  /** Set the intake motor power in the range of [-1, 1]. */
  private void setIntakePower(double power) {
    intakeMotor.set(power);
  }

  /** Set the conveyor motor power in the range of [-1, 1]. */
  private void setConveyorPower(double power) {
    conveyorMotor.set(power);
  }

  /** Set the pivot target angle in degrees. */
  private void setPivotPosition(double degrees) {
    pivotController.setReference(degrees, ControlType.kPosition);
  }

  /**
   * Command to run the intake and conveyor motors. When the command is interrupted, e.g. the button is released,
   * the motors will stop.
   */
  public Command runIntakeCommand() {
    return this.startEnd(
        () -> {
          this.setIntakePower(IntakeSetpoints.kIntake);
          this.setConveyorPower(ConveyorSetpoints.kIntake);
        }, () -> {
          this.setIntakePower(0.0);
          this.setConveyorPower(0.0);
        }).withName("Intaking");
  }

  /**
   * Command to reverse the intake motor and coveyor motors. When the command is interrupted, e.g. the button is
   * released, the motors will stop.
   */
  public Command runExtakeCommand() {
    return this.startEnd(
        () -> {
          this.setIntakePower(IntakeSetpoints.kExtake);
          this.setConveyorPower(ConveyorSetpoints.kExtake);
        }, () -> {
          this.setIntakePower(0.0);
          this.setConveyorPower(0.0);
        }).withName("Extaking");
  }

  /**
   * Command to run the intake and conveyor motors for a set time.
   * @param seconds The time to run the intake for.
   * NOT USING YET, i'm thinking we could use this in auto..
   * ..naviate somewhere then intake for x seconds.
   */
  public Command runIntakeForTime(double seconds) {
    return runIntakeCommand()
      .withTimeout(seconds)
      .withName(String.format("Intaking for %.2fs", seconds));
  }

  /**
   * Command to run just the conveyor motor.
   */
  public Command runConveyorCommand() {
    return this.startEnd(
        () -> this.setConveyorPower(ConveyorSetpoints.kIntake),
        () -> this.setConveyorPower(0.0)
    ).withName("Run Conveyor");
  }

  /**
   * Command to deploy the intake.
   */
  public Command deployIntakeCommand() {
    return this.runOnce(() -> setPivotPosition(PivotSetpoints.kDeployedDegrees))
        .withName("Deploy Intake");
  }

  /**
   * Command to retract the intake.
   */
  public Command retractIntakeCommand() {
    return this.runOnce(() -> setPivotPosition(PivotSetpoints.kRetractedDegrees))
        .withName("Retract Intake");
  }

  @Override
  public void periodic() {
    // Display subsystem values
    SmartDashboard.putNumber("Intake | Intake | Applied Output", intakeMotor.getAppliedOutput());
    SmartDashboard.putNumber("Intake | Conveyor | Applied Output", conveyorMotor.getAppliedOutput());
    SmartDashboard.putNumber("Intake | Pivot | Position (Deg)", pivotEncoder.getPosition());
  }

}
