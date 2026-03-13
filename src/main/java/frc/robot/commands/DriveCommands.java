// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;

import static edu.wpi.first.units.Units.Inches;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 10.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 30.0;
  private static final double ANGLE_MAX_ACCELERATION = 30.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  public static Optional<Pose2d> targetAimPose = Optional.empty();
  public static final Pose2d centerField = new Pose2d(Inches.of(325.61), Inches.of(158.84), Rotation2d.kZero);
  public static final Pose2d blueGoal = new Pose2d(Inches.of(182.11), Inches.of(158.84), Rotation2d.kZero);
  public static final Pose2d redGoal = blueGoal.rotateAround(centerField.getTranslation(), Rotation2d.k180deg);

  private DriveCommands() {
  }

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and
   * angular velocities).
   * @param drive drivebase
   * @param xSupplier supplies velocity in field x  direction
   * @param ySupplier supplies velocity in field y direction
   * @param omegaSupplier supplies velocity for robot rotation
   * @return Command for driving
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
              ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds = new ChassisSpeeds(
              linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
              linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
              omega * drive.getMaxAngularSpeedRadPerSec());
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped()
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  /**
   * Robot relative drive command using two joysticks (controlling linear and
   * angular velocities).
   * @param drive drivebase
   * @param xSupplier supplies velocity in robot direction
   * @param ySupplier supplies velocity in robot direction
   * @param omegaSupplier supplies velocity for robot rotation
   * @return Command for driving
   */
  public static Command joystickDriveRobot(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
              ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds = new ChassisSpeeds(
              linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
              linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
              omega * drive.getMaxAngularSpeedRadPerSec());
          drive.runVelocity(speeds);
        },
        drive);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for
   * angular control. Possible use cases include snapping to an angle, aiming at a
   * vision target, or controlling absolute rotation with a joystick.
   * 
   * @param drive drivebase
   * @param xSupplier supplies velocity in field x direction
   * @param ySupplier supplies velocity in field y direction
   * @param rotationSupplier gives angle to drive at (0 degrees is away from blue alliance station).
   * @return Command for driving
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController = new ProfiledPIDController(
        ANGLE_KP,
        0.0,
        ANGLE_KD,
        new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
              ySupplier.getAsDouble());

          // Calculate angular speed
          double omega = angleController.calculate(
              drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds = new ChassisSpeeds(
              linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
              linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
              omega);
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped()
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  /**
   * Is the field flipped (because on red alliance)
   * 
   * @return is on red Alliance
   */
  public static boolean isFlipped() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
  }

  /**
   * Field Relative Drive at angle commanded by combo of two angle suppliers (such as a joystick x and y)
   * @param drive
   * @param xVelSupplier
   * @param yVelSupplier
   * @param xAngleSupplier
   * @param yAngleSupplier
   * @return
   */
  public static Command joystickDriveAbsoluteAngle (
      Drive drive,
      DoubleSupplier xVelSupplier,
      DoubleSupplier yVelSupplier,
      DoubleSupplier xAngleSupplier,
      DoubleSupplier yAngleSupplier
      )
      {
        
    Supplier<Rotation2d> angle = () -> {
      double x = xAngleSupplier.getAsDouble();
      double y = yAngleSupplier.getAsDouble();
      //check deadband
      x = MathUtil.applyDeadband(x, DEADBAND);
      y = MathUtil.applyDeadband(y, DEADBAND);
      if (x ==0 && y == 0 ){
        return drive.getRotation(); // in deadband, shouldn't be turning.
      }
      return new Rotation2d(Math.atan2(y, x)).rotateBy(isFlipped()?Rotation2d.k180deg:Rotation2d.kZero);
    };
        return joystickDriveAtAngle(drive, xVelSupplier, yVelSupplier, angle);
      }
  /**
   * Field relative drive command using joystick for linear control 
   * Rotates to follow drivebase velocity vector. 
   * 
   * @param drive drivebase
   * @param xSupplier supplies velocity in field x direction
   * @param ySupplier supplies velocity in field y direction
   * @return Command for driving
   */
  public static Command joystickDriveSnake(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {

    return joystickDriveAbsoluteAngle(drive, xSupplier, ySupplier, xSupplier, ySupplier);

  }

  /**
   * @return Pose2d of goal for current alliance
   */
  public static Pose2d getAllianceGoal() {
    return isFlipped() ? redGoal : blueGoal;
  }

  /**
   * sets the targetAimPose to the appropriate goal
   */
  public static void setGoalTarget() {
    targetAimPose = Optional.of(getAllianceGoal());
    Logger.recordOutput("targetAimPose", targetAimPose.get());
  }

  public static Command setGoalTargetCommand() {
    return Commands.runOnce(() -> setGoalTarget());
  }

  /**
   * Drives while aimed at target, field relative.
   * 
   * uses Pose2d targetAimPose for target location. defaults to allianceGoal if target not set.
   * 
   * @param drive drivebase
   * @param xSupplier supplies velocity in field x direction
   * @param ySupplier supplies velocity in field y direction
   * @return Command for driving
   */

  public static Command joystickDriveAim(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {

    Supplier<Rotation2d> angle = () -> drive.getPose().getTranslation()
        .minus(
          targetAimPose.orElseGet(DriveCommands::getAllianceGoal) // default to alliance goal
          .getTranslation()).getAngle().plus(Rotation2d.k180deg);
    ;
    return joystickDriveAtAngle(drive, xSupplier, ySupplier, angle);
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>
   * This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
            () -> {
              drive.runCharacterization(0.0);
            },
            drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
            () -> {
              double voltage = timer.get() * FF_RAMP_RATE;
              drive.runCharacterization(voltage);
              velocitySamples.add(drive.getFFCharacterizationVelocity());
              voltageSamples.add(voltage);
            },
            drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                () -> {
                  var rotation = drive.getRotation();
                  state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                  state.lastAngle = rotation;
                })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * Constants.DriveConstants.driveBaseRadius) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }
}
