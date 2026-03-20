package frc.robot.subsystems.intake;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import frc.robot.Constants.Intake;


/** Simulation IO for the Intake using SparkFlexSim and a SingleJointedArmSim for the pivot */
public class IntakeIOSim implements IntakeIO {
  // Motors
  private final SparkFlex intakeSpark = new SparkFlex(Intake.kIntakeMotorCanId, MotorType.kBrushless);
  private final SparkFlex conveyorSpark = new SparkFlex(Intake.kConveyorMotorCanId, MotorType.kBrushless);
  private final SparkFlex pivotSpark = new SparkFlex(Intake.kPivotMotorCanId, MotorType.kBrushless);

  private final SparkFlexSim intakeSim = new SparkFlexSim(intakeSpark, DCMotor.getNeoVortex(1));
  private final SparkFlexSim conveyorSim = new SparkFlexSim(conveyorSpark, DCMotor.getNeoVortex(1));
  private final SparkFlexSim pivotSim = new SparkFlexSim(pivotSpark, DCMotor.getNeoVortex(1));

  // Arm sim for the pivot. We pick reasonable defaults for length/mass/gearing.
  private static final double ARM_LENGTH_METERS = 0.254; // 10 inches
  private static final double ARM_MASS_KG = 1.0; // 1 kg
  // Use a modest gearing (motor -> arm) to match configs pivot conversion factor; use 36:1 as in Configs comments
  private static final double GEARING = 36.0;
  // The authoritative arm sim using WPILib's SingleJointedArmSim
  private final SingleJointedArmSim armSim;

  private double lastTime = Timer.getFPGATimestamp();

  public IntakeIOSim() {
    // Use WPILib's SingleJointedArmSim constructor that takes (DCMotor, gearing, jKgMetersSquared,
    // armLengthMeters, minAngleRads, maxAngleRads, simulateGravity, startingAngleRads)
    double moi = SingleJointedArmSim.estimateMOI(ARM_LENGTH_METERS, ARM_MASS_KG);
    // Limit pivot between 0 deg (retracted) and 90 deg (deployed)
    double minAngle = 0.0;
    double maxAngle = Math.PI / 2.0;
    armSim = new SingleJointedArmSim(
        DCMotor.getNeoVortex(1),
        GEARING,
        moi,
        ARM_LENGTH_METERS,
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

    // Iterate spark sims so controllers update their internal state
    intakeSim.iterate(now, dt, battery);
    conveyorSim.iterate(now, dt, battery);
    pivotSim.iterate(now, dt, battery);

    // Intake & conveyor: read applied voltage/current from the Spark
    inputs.intakeAppliedVoltage = intakeSpark.getAppliedOutput() * battery;
    inputs.conveyorAppliedVoltage = conveyorSpark.getAppliedOutput() * battery;

    // Pivot: drive arm sim with motor applied voltage
    double appliedVolts = pivotSpark.getAppliedOutput() * battery;
    armSim.setInputVoltage(appliedVolts);
    armSim.update(dt);
    double angleRad = armSim.getAngleRads();
    // Map radians to absolute encoder values for simulation (0 rad = Deployed, 90 deg = Retracted)
    double absPos = Intake.PivotSetpoints.kDeployedPosition + (angleRad / (Math.PI / 2.0)) * (Intake.PivotSetpoints.kRetractedPosition - Intake.PivotSetpoints.kDeployedPosition);

    // Publish pivot state
    inputs.pivotPosition = absPos;
    inputs.pivotVelocity = armSim.getVelocityRadPerSec();
    inputs.pivotAppliedVoltage = appliedVolts;
    inputs.pivotCurrent = Math.abs(armSim.getCurrentDrawAmps());
    // Try to read target setpoint from the pivot Spark's closed loop controller
    try {
      inputs.pivotTargetPosition = pivotSpark.getClosedLoopController().getSetpoint();
    } catch (Exception ignored) {}

    // Push simulated encoder position back into the SparkFlex external encoder sim so the controller can read it
    try {
      var encSim = pivotSim.getAbsoluteEncoderSim();
      encSim.setPosition(absPos);
      encSim.setVelocity(armSim.getVelocityRadPerSec());
    } catch (Exception ignored) {
    }
  }

  @Override
  public void setPivotPosition(double degrees) {
    try {
      pivotSpark.getClosedLoopController().setSetpoint(degrees, com.revrobotics.spark.SparkBase.ControlType.kPosition);
    } catch (Exception ignored) {
    }
  }

  @Override
  public void setPivotDutyCycle(double dutyCycle) {
    try { pivotSpark.set(dutyCycle); } catch (Exception ignored) {}
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
    try { intakeSpark.stopMotor(); } catch (Exception ignored) {}
    try { conveyorSpark.stopMotor(); } catch (Exception ignored) {}
    try { pivotSpark.stopMotor(); } catch (Exception ignored) {}
  }
}
