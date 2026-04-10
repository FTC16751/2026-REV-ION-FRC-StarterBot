// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
  // TODO: set CAN ids to match code

  public static final class Intake {
    public static final int kIntakeMotorCanId = 31; // SPARK Flex CAN ID
    public static final int kIntakeFollowerMotorCanId = 32; // SPARK Flex CAN ID (follower)
    public static final int kPivotMotorCanId = 30; // SPARK Flex CAN ID (New Pivot Motor)
    public static final int kPivotFollowerMotorCanId = 33; // SPARK Flex CAN ID (Right Pivot Motor, follower)

    public static final class IntakeSetpoints {
      public static final double kIntake = 0.7;
      public static final double kExtake = -0.5;
      public static final double kMaxPower = .7;
    }

    public static final class PivotSetpoints {
      public static final double kZeroOffset = 0.5; 
      public static final double kRetractedPosition = 0.42;
      public static final double kDeployedPosition = 0.050;
      // Cosine feedforward constant for gravity-hold tuning.
      public static final double kCos = 0.25;
      public static final double kPositionTolerance = 0.02; // rotations — PID at-target threshold
      // How close the arm must be to kDeployedPosition before free-deploy mode engages.
      public static final double kCompliantHoldTolerance = 0.01;
      // Nudge: rotations added to pivotTarget per 20ms cycle while button held (~0.25 rot/sec)
      public static final double kNudgePerCycle = 0.005;
      public static final double kAgitatePosition = 0.1500; // Upper agitate position
    }

    // Arm sim for the pivot. We pick reasonable defaults for length/mass/gearing.
    public static final double ARM_LENGTH_METERS = 0.254; // 10 inches
    public static final double ARM_MASS_KG = 1.0; // 1 kg
    // Use a modest gearing (motor -> arm) to match configs pivot conversion factor; use 36:1 as in Configs comments
    public static final double GEARING = 36.0;
  }

  public static final class Conveyor {
    public static final int kConveyorMotorCanId = 23; // SPARK Flex CAN ID
    
    public static final class ConveyorSetpoints {
      public static final double kConveyorSpeedLaunch     = 1.0;  // full speed — used during launch
      public static final double kConveyorSpeedIntake = 0.4;  // slower speed — used while intaking
      public static final double kConveyorExtakeSpeed     = -0.7;
    }
  }

  public static final class ShooterSubsystemConstants {
    public static final int kFeederMotorCanId = 20; // SPARK Flex CAN ID
    public static final int kFlywheelMotorCanId = 22; // SPARK Flex CAN ID (Right)
    public static final int kFlywheelFollowerMotorCanId = 21; // SPARK Flex CAN ID (Left)

    public static final class FeederSetpoints {
      public static final double kFeed = 1.00;
    }

    public static final class FlywheelSetpoints {
      public static final double kShootRpm = 2500;
      public static final double kMaxRpm = 6000; // hard cap for operator speed adjustments
      public static final double kIdleRpm = 1000; // coasting speed to reduce spin-up time
      public static final double kVelocityTolerance = 100;
    }
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians pe
    public static final double odometryFrequency = 100.0; // Hzr second

    // Chassis configuration
    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = Units.inchesToMeters(22.5);
    // Distance between front and back wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(22.5);

    public static final double driveBaseRadius = Math.hypot(kTrackWidth / 2.0, kWheelBase / 2.0);
    public static final Translation2d[] moduleTranslations = new Translation2d[] {
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
    };

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(moduleTranslations);

    // The EasySwerve module allows installation of the motors either on top or
    // bottom of the module.
    // These constants configure the location of the motors. The default
    // configuration is with both
    // motors on the bottom of the module.
    public static final boolean kFrontLeftDrivingMotorOnBottom = true;
    public static final boolean kRearLeftDrivingMotorOnBottom = true;
    public static final boolean kFrontRightDrivingMotorOnBottom = true;
    public static final boolean kRearRightDrivingMotorOnBottom = true;

    public static final boolean kFrontLeftTurningMotorOnBottom = true;
    public static final boolean kRearLeftTurningMotorOnBottom = true;
    public static final boolean kFrontRightTurningMotorOnBottom = true;
    public static final boolean kRearRightTurningMotorOnBottom = true;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 10;
    public static final int kRearLeftDrivingCanId = 14;
    public static final int kFrontRightDrivingCanId = 12;
    public static final int kRearRightDrivingCanId = 16;

    public static final int kFrontLeftTurningCanId = 11;
    public static final int kRearLeftTurningCanId = 15;
    public static final int kFrontRightTurningCanId = 13;
    public static final int kRearRightTurningCanId = 17;

    // Zeroed rotation values for each module, see setup instructions
    public static final Rotation2d frontLeftZeroRotation = new Rotation2d(Degrees.of(270));
    public static final Rotation2d frontRightZeroRotation = new Rotation2d(Degrees.of(0));
    public static final Rotation2d backLeftZeroRotation = new Rotation2d(Degrees.of(180));
    public static final Rotation2d backRightZeroRotation = new Rotation2d(Degrees.of(90));

    public static final boolean kGyroReversed = false;

    // PathPlanner configuration
    public static final double robotMassKg = 74.088;
    public static final double robotMOI = 6.883;
    public static final double wheelCOF = 1.2;
    public static final RobotConfig ppConfig = new RobotConfig(
        robotMassKg,
        robotMOI,
        new ModuleConfig(
            ModuleConstants.wheelRadiusMeters,
            kMaxSpeedMetersPerSecond,
            wheelCOF,
            ModuleConstants.driveGearbox.withReduction(ModuleConstants.kDrivingMotorReduction),
            ModuleConstants.driveMotorCurrentLimit,
            1),
        moduleTranslations);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
    public static final double kVortexKv = 565; // rpm/V
  }

  public static final class ModuleConstants { 
    // The EasySwerve module can only be configured with one pinion gears:
    public static final int kDrivingMotorPinionTeeth = 12;

    public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1);

    public static final int driveMotorCurrentLimit = 50;
    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3);
    public static final double wheelRadiusMeters = kWheelDiameterMeters / 2;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 30 teeth on the first-stage spur gear,
    // 15 teeth on the bevel pinion
    public static final double kDrivingWheelBevelGearTeeth = 45.0;
    public static final double kDrivingWheelFirstStageSpurGearTeeth = 22.0;
    public static final double kDrivingMotorBevelPinionTeeth = 15.0;
    public static final double kDrivingMotorReduction = 
      (kDrivingWheelBevelGearTeeth * kDrivingWheelFirstStageSpurGearTeeth) / (kDrivingMotorPinionTeeth * kDrivingMotorBevelPinionTeeth);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    // Drive encoder configuration
    public static final double driveEncoderPositionFactor = 2 * Math.PI / kDrivingMotorReduction; // Rotor Rotations ->
    // Wheel Radians
    public static final double driveEncoderVelocityFactor = (2 * Math.PI) / 60.0 / kDrivingMotorReduction; // Rotor RPM
                                                                                                           // ->
    // Wheel Rad/Sec
    // Drive PID configuration
    public static final double driveKp = 0.0;
    public static final double driveKd = 0.0;
    public static final double driveKs = 0.0;
    public static final double driveKv = 0.1;
    public static final double driveSimP = 0.05;
    public static final double driveSimD = 0.0;
    public static final double driveSimKs = 0.0;
    public static final double driveSimKv = 0.0789;

    // Turn motor configuration
    public static final boolean turnInverted = false;
    public static final int turnMotorCurrentLimit = 40;
    public static final double turnMotorReduction = 25.0;
    public static final DCMotor turnGearbox = DCMotor.getNeoVortex(1);

    // Turn encoder configuration
    public static final boolean turnEncoderInverted = true;
    public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
    public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

    // Turn PID configuration
    public static final double turnKp = 2.0;
    public static final double turnKd = 0.0;
    public static final double turnSimP = 8.0;
    public static final double turnSimD = 0.0;
    public static final double turnPIDMinInput = 0; // Radians
    public static final double turnPIDMaxInput = 2 * Math.PI; // Radians
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.1;
    public static final double kTriggerButtonThreshold = 0.2;
    public static final double kNormalSpeedMultiplier    = 1.0;
    public static final double kSlowSpeedMultiplier      = 0.50;
    public static final double kSuperSlowSpeedMultiplier = 0.25; // LB + RB held simultaneously
    public static final double kFastSpeedMultiplier      = 0.8;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

}
