package frc.robot.subsystems.intake;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import frc.robot.Constants.IntakeSubsystemConstants;

/**
 * Simulation IO for the Intake using SparkFlexSim and a SingleJointedArmSim for
 * the pivot
 */
public class IntakeIOSim implements IntakeIO {
  // Motors
  private final SparkFlex intakeSpark = new SparkFlex(IntakeSubsystemConstants.kIntakeMotorCanId, MotorType.kBrushless);
  private final SparkFlex conveyorSpark = new SparkFlex(IntakeSubsystemConstants.kConveyorMotorCanId,
      MotorType.kBrushless);
  private final SparkFlex pivotSpark = new SparkFlex(IntakeSubsystemConstants.kPivotMotorCanId, MotorType.kBrushless);

  private final SparkFlexSim intakeSim = new SparkFlexSim(intakeSpark, DCMotor.getNeoVortex(1));
  private final SparkFlexSim conveyorSim = new SparkFlexSim(conveyorSpark, DCMotor.getNeoVortex(1));
  private final SparkFlexSim pivotSim = new SparkFlexSim(pivotSpark, DCMotor.getNeoVortex(1));

  // The authoritative arm sim using WPILib's SingleJointedArmSim
  private final SingleJointedArmSim armSim;

  private double lastTime = Timer.getFPGATimestamp();

  public IntakeIOSim() {
    // Use WPILib's SingleJointedArmSim constructor that takes (DCMotor, gearing,
    // jKgMetersSquared,
    // armLengthMeters, minAngleRads, maxAngleRads, simulateGravity,
    // startingAngleRads)
    double moi = SingleJointedArmSim.estimateMOI(IntakeSubsystemConstants.ARM_LENGTH_METERS,
        IntakeSubsystemConstants.ARM_MASS_KG);
    // Limit pivot between 0 deg (retracted) and 90 deg (deployed)
    double minAngle = 0.0;
    double maxAngle = Math.PI / 2.0;
    armSim = new SingleJointedArmSim(
        DCMotor.getNeoVortex(1),
        IntakeSubsystemConstants.GEARING,
        moi,
        IntakeSubsystemConstants.ARM_LENGTH_METERS,
        minAngle,
        maxAngle,
        true,
        0.0);
  }

  @Override
  public void updateInputs(IntakeIO.IntakeIOInputs inputs) {
    double now = Timer.getFPGATimestamp();
    double dt = Math.max(0.0, now - lastTime);
    lastTime = now;

    double battery = RobotController.getBatteryVoltage();

    // First, iterate intake & conveyor sims so controllers update their internal state
    intakeSim.iterate(now, dt, battery);
    conveyorSim.iterate(now, dt, battery);

    // Drive arm sim with motor applied voltage from the last spark sim iteration
    double appliedVoltsPrev = pivotSpark.getAppliedOutput() * battery;
    armSim.setInputVoltage(appliedVoltsPrev);
    armSim.update(dt);
    double angleRad = armSim.getAngleRads();
    double angleDeg = Math.toDegrees(angleRad);

    // Publish the measured encoder position/velocity to the Spark sim BEFORE
    // iterating the pivot Spark sim so the closed-loop controller sees the
    // current plant state when it computes outputs for this timestep.
    var encSim = pivotSim.getExternalEncoderSim();
    // The Spark encoder is configured to report degrees (Configs sets
    // positionConversionFactor accordingly)
    encSim.setPosition(angleDeg);
    // Convert arm sim velocity (rad/s) to degrees/sec for the encoder sim
    encSim.setVelocity(Math.toDegrees(armSim.getVelocityRadPerSec()));

    // Now iterate pivot sim so its closed-loop controller reads the encoder we
    // just wrote and computes an output for this timestep.
    pivotSim.iterate(now, dt, battery);

    // Read applied voltage/current from the Spark
    inputs.intakeAppliedVoltage = intakeSpark.getAppliedOutput() * battery;
    inputs.conveyorAppliedVoltage = conveyorSpark.getAppliedOutput() * battery;

    double appliedVolts = pivotSpark.getAppliedOutput() * battery;

    // Publish pivot state
    inputs.pivotPosition = angleDeg;
    inputs.pivotAppliedVoltage = appliedVolts;
    inputs.pivotCurrent = Math.abs(armSim.getCurrentDrawAmps());
    try {
      inputs.pivotTargetPosition = pivotSpark.getClosedLoopController().getSetpoint();
    } catch (Exception e) {
      inputs.pivotTargetPosition = 0.0;
    }
  }

  @Override
  public void setPivotPosition(double degrees) {
      pivotSpark.getClosedLoopController().setSetpoint(degrees, com.revrobotics.spark.SparkBase.ControlType.kPosition);

  }

  @Override
  public void setIntakePower(double power) {
    intakeSpark.set(power);
  }

  @Override
  public void setConveyorPower(double power) {
    conveyorSpark.set(power);
  }

  @Override
  public void stop() {
      intakeSpark.stopMotor();
      conveyorSpark.stopMotor();
      pivotSpark.stopMotor();
  }
}
