package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radian;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.Intake;

/**
 * Simulation IO for the Intake using SparkFlexSim and a SingleJointedArmSim for
 * the pivot
 */
public class IntakeIOSim implements IntakeIO {
  // Motors
  private final SparkFlex intakeSpark = new SparkFlex(Intake.kIntakeMotorCanId, MotorType.kBrushless);
  private final SparkFlex conveyorSpark = new SparkFlex(Intake.kConveyorMotorCanId,
      MotorType.kBrushless);
  private final SparkFlex pivotMotor = new SparkFlex(Intake.kPivotMotorCanId, MotorType.kBrushless);

  private final SparkFlexSim intakeSim = new SparkFlexSim(intakeSpark, DCMotor.getNeoVortex(1));
  private final SparkFlexSim conveyorSim = new SparkFlexSim(conveyorSpark, DCMotor.getNeoVortex(1));
  private final SparkFlexSim pivotSim = new SparkFlexSim(pivotMotor, DCMotor.getNeoVortex(1));

  // Simple flywheel sims for intake and conveyor so the motors show
  // output/current
  private static final double FLYWHEEL_MOI = 0.001; // kg*m^2 as requested
  private static final double FLYWHEEL_GEARING = 1.0;
  private final FlywheelSim intakeFlywheelSim = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(DCMotor.getNeoVortex(1), FLYWHEEL_MOI, FLYWHEEL_GEARING),
      DCMotor.getNeoVortex(1));
  private final FlywheelSim conveyorFlywheelSim = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(DCMotor.getNeoVortex(1), FLYWHEEL_MOI, FLYWHEEL_GEARING),
      DCMotor.getNeoVortex(1));

  // The authoritative arm sim using WPILib's SingleJointedArmSim
  private final SingleJointedArmSim armSim;

  private double lastTime = Timer.getFPGATimestamp();
  private SparkRelativeEncoderSim pivotEncoder = pivotSim.getRelativeEncoderSim();

  public IntakeIOSim() {
    // Use WPILib's SingleJointedArmSim constructor that takes (DCMotor, gearing,
    // jKgMetersSquared,
    // armLengthMeters, minAngleRads, maxAngleRads, simulateGravity,
    // startingAngleRads)
    double moi = SingleJointedArmSim.estimateMOI(Intake.ARM_LENGTH_METERS,
        Intake.ARM_MASS_KG);
    // Limit pivot between 0 deg (retracted) and 90 deg (deployed)
    double minAngle = 0.0;
    double maxAngle = Radian.convertFrom(Constants.Intake.PivotSetpoints.kRetractedDegrees,Degrees);
    armSim = new SingleJointedArmSim(
        DCMotor.getNeoVortex(1),
        Intake.GEARING,
        moi,
        Intake.ARM_LENGTH_METERS,
        minAngle,
        maxAngle,
        true,
        maxAngle);

    // Configure simulated Spark controllers with the team Configs so closed-loop
    // controllers and encoder conversions match real robot settings.
    intakeSpark.configure(
        Configs.IntakeSubsystem.intakeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    conveyorSpark.configure(
        Configs.IntakeSubsystem.conveyorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    pivotMotor.configure(
        Configs.IntakeSubsystem.pivotConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    pivotEncoder.setPosition(Constants.Intake.PivotSetpoints.kRetractedDegrees);
  }

  @Override
  public void updateInputs(IntakeIO.IntakeIOInputs inputs) {
    // calc dt
    double now = Timer.getFPGATimestamp();
    double dt = Math.max(0.0, now - lastTime);
    lastTime = now;

    double iterateBattery = RoboRioSim.getVInVoltage();

    // Read applied outputs (from previous controller step) and drive physics models
    intakeFlywheelSim.setInputVoltage(intakeSpark.getAppliedOutput() * iterateBattery);
    intakeFlywheelSim.update(dt);
    double intakeRotorRadPerSec = intakeFlywheelSim.getAngularVelocityRadPerSec();

    conveyorFlywheelSim.setInputVoltage(conveyorSpark.getAppliedOutput() * iterateBattery);
    conveyorFlywheelSim.update(dt);
    double conveyorRotorRadPerSec = conveyorFlywheelSim.getAngularVelocityRadPerSec();

    // Drive arm sim with motor applied voltage from the last controller step
    armSim.setInputVoltage(pivotMotor.getAppliedOutput() * iterateBattery);
    armSim.update(dt);
    double angleDeg = Math.toDegrees(armSim.getAngleRads());

    // Push simulated encoder state into the Spark encoder sims
    // FlywheelSim returns rad/s for rotor velocity
    intakeSim.getRelativeEncoderSim().setVelocity(intakeRotorRadPerSec);
    conveyorSim.getRelativeEncoderSim().setVelocity(conveyorRotorRadPerSec);

    // The Spark encoder for pivot is configured to report degrees
    pivotEncoder.setPosition(angleDeg);
    pivotEncoder.setVelocity(Math.toDegrees(armSim.getVelocityRadPerSec()));

    // Now iterate the Spark sims so their controllers read the encoder state we
    // just wrote and compute outputs for this timestep. Pass the simulated
    // rotor/encoder velocities for each motor.
    intakeSim.iterate(intakeRotorRadPerSec, iterateBattery, dt);
    conveyorSim.iterate(conveyorRotorRadPerSec, iterateBattery, dt);
    pivotSim.iterate(Math.toDegrees(armSim.getVelocityRadPerSec()), iterateBattery, dt);

    // Read applied current from sims and update battery loaded voltage
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps(),
        Math.abs(intakeFlywheelSim.getCurrentDrawAmps()), Math.abs(conveyorFlywheelSim.getCurrentDrawAmps())));

    // Publish inputs (use the iterateBattery-driven applied voltages we computed)
    inputs.intakeAppliedVoltage = intakeSpark.getAppliedOutput() * iterateBattery;
    inputs.conveyorAppliedVoltage = conveyorSpark.getAppliedOutput() * iterateBattery;
    inputs.pivotPosition = angleDeg;
    inputs.pivotVelocity = pivotSim.getVelocity();
    inputs.pivotAppliedVoltage = pivotMotor.getAppliedOutput() * iterateBattery;
    inputs.pivotCurrent = Math.abs(armSim.getCurrentDrawAmps());
    inputs.pivotTargetPosition = pivotMotor.getClosedLoopController().getSetpoint();
  }

  @Override
  public void setPivotPosition(double degrees) {
    pivotMotor.getClosedLoopController().setSetpoint(degrees, com.revrobotics.spark.SparkBase.ControlType.kPosition);

  }

  @Override
  public void setIntakePower(double power) {
    intakeSpark.set(power);
  }

  

  @Override
  public void setIntakeSpeed(AngularVelocity speed) {
    intakeSpark.getClosedLoopController().setSetpoint(speed.in(RPM), ControlType.kVelocity);
  }

  @Override
  public void setConveyorPower(double power) {
    conveyorSpark.set(power);
  }

  @Override
  public void zeroPivotPosition(boolean bottom){
    pivotMotor.getEncoder().setPosition(bottom ? Constants.Intake.PivotSetpoints.kDeployedDegrees : Constants.Intake.PivotSetpoints.kRetractedDegrees);
  }

  @Override
  public void stop() {
    intakeSpark.stopMotor();
    conveyorSpark.stopMotor();
    pivotMotor.stopMotor();
  }
}
