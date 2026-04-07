// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import com.pathplanner.lib.auto.NamedCommands;

import com.pathplanner.lib.auto.AutoBuilder;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.Autos;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.led.LEDSubsystem.LEDState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.ConveyorIO;
import frc.robot.subsystems.conveyor.ConveyorIOSim;
import frc.robot.subsystems.conveyor.ConveyorIOSpark;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.RobotController;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems and commands are defined here...
        public Vision vision;
        public Drive drive;
        public Shooter m_shooter;

        public Intake m_intake;
        public Conveyor m_conveyor;
        private static RobotContainer instance;
        private final LEDSubsystem m_leds = new LEDSubsystem();

    // The driver's controller
    private final CommandXboxController driveCtrlr = new CommandXboxController(OIConstants.kDriverControllerPort);
    // The operator's controller (all non-driving mechanisms)
    private final CommandXboxController operatorCtrlr = new CommandXboxController(OIConstants.kOperatorControllerPort);

    private final LoggedDashboardChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        instance = this;

        switch (Constants.currentMode) {
            case REAL:
                drive = new Drive(
                        new GyroIONavX(),
                        new ModuleIOSpark(0),
                        new ModuleIOSpark(1),
                        new ModuleIOSpark(2),
                        new ModuleIOSpark(3));
                m_shooter = new Shooter(new ShooterIOSpark(), () -> drive.distanceToTarget().in(Meters));
                m_intake = new Intake(new frc.robot.subsystems.intake.IntakeIOSpark());
                m_conveyor = new Conveyor(new ConveyorIOSpark());
                vision = new Vision(
                        drive::addVisionMeasurement,
                        new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation));
                break;

                        case SIM:
                                drive = new Drive(
                                                new GyroIO() {
                                                },
                                                new ModuleIOSim(),
                                                new ModuleIOSim(),
                                                new ModuleIOSim(),
                                                new ModuleIOSim());
                                m_shooter = new Shooter(new ShooterIOSim(), () -> drive.distanceToTarget().in(Meters));
                                m_intake = new Intake(new frc.robot.subsystems.intake.IntakeIOSim());
                                m_conveyor = new Conveyor(new ConveyorIOSim());
                                vision = new Vision(drive::addVisionMeasurement, new VisionIO() {
                                });
                                break;

                        default:
                                drive = new Drive(
                                                new GyroIO() {
                                                },
                                                new ModuleIO() {
                                                },
                                                new ModuleIO() {
                                                },
                                                new ModuleIO() {
                                                },
                                                new ModuleIO() {
                                                });
                                m_shooter = new Shooter(new ShooterIO() {
                                }, () -> drive.distanceToTarget().in(Meters));
                                m_intake = new Intake(new frc.robot.subsystems.intake.IntakeIO() {
                                });
                                m_conveyor = new Conveyor(new ConveyorIO() {
                                });
                                vision = new Vision(drive::addVisionMeasurement, new VisionIO() {
                                });
                                break;
                }

                // Register Named Commands for PathPlanner
                NamedCommands.registerCommand("DeployIntake", m_intake.deployIntakeCommand());
                NamedCommands.registerCommand("RetractIntake", m_intake.retractIntakeCommand());
                NamedCommands.registerCommand("RunIntake", m_intake.runIntakeCommand()
                        .withTimeout(3.0));
                // Recombine intake and conveyor for PathPlanner
                NamedCommands.registerCommand("RunIntake_withConveyor", m_intake.runIntakeCommand().alongWith(m_conveyor.runConveyorIntakeCommand())
                        .withTimeout(3.0));
                // Re-use the same reliable shooting sequence we built for the Y button
                NamedCommands.registerCommand("Shoot", m_shooter.runShooterCommand()
                        .alongWith(Commands.waitUntil(m_shooter.isFlywheelSpinning)
                                .andThen(m_conveyor.runConveyorCommand()))
                        .withTimeout(7.0)); // Timeout ensures auto doesn't hang forever
                NamedCommands.registerCommand("AutoAim",
                        DriveCommands.joystickDriveAim(drive, () -> 0.0, () -> 0.0)
                                .until(() -> drive.isAimedAtTarget(3.0))
                                .withTimeout(2.0));
                NamedCommands.registerCommand("AimAndShoot",
                        m_shooter.enableAutoSpeedCommand()
                                .andThen(DriveCommands.joystickDriveAim(drive, () -> 0.0, () -> 0.0)
                                        .until(() -> drive.isAimedAtTarget(3.0))
                                        .withTimeout(2.0))
                                .andThen(m_shooter.runShooterCommand()
                                        .alongWith(Commands.waitUntil(m_shooter.isFlywheelSpinning)
                                                .andThen(m_conveyor.runConveyorCommand()))
                                        .withTimeout(7.0)));
                NamedCommands.registerCommand("AgitateIntake", m_intake.agitateCommand());
                NamedCommands.registerCommand("AimShootAndShimmy",
                        m_shooter.enableAutoSpeedCommand()
                                .andThen(
                                        // Rotate to aim
                                        DriveCommands.joystickDriveAim(drive, () -> 0.0, () -> 0.0)
                                                .until(() -> drive.isAimedAtTarget(3.0))
                                                .withTimeout(2.0))
                                .andThen(
                                        // Shimmy + shoot simultaneously
                                         m_shooter.runShooterCommand()
                                                .alongWith(Commands.waitUntil(m_shooter.isFlywheelSpinning)
                                                        .andThen(m_conveyor.runConveyorCommand()
                                                                .alongWith(DriveCommands.joystickDriveAim(drive,
                                                                        () -> Math.sin(Timer.getFPGATimestamp() * Math.PI) * 0.12,
                                                                        () -> 0.0))
                                                                .alongWith(m_intake.agitateCommand())
                                                        )
                                                )
                                                .withTimeout(7.0)));

                    // Set up auto routines
                    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

                    // Add Custom Autos
                    autoChooser.addOption("Simple Auto", Autos.simpleAuto(m_shooter, m_intake, m_conveyor));

                    // Set up SysId routines
                    autoChooser.addOption(
                                        "Drive Wheel Radius Characterization",
                                DriveCommands.wheelRadiusCharacterization(drive));
                    autoChooser.addOption(
                                        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
                    autoChooser.addOption(
                                        "Drive SysId (Quasistatic Forward)",
                                        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
                    autoChooser.addOption(
                                        "Drive SysId (Quasistatic Reverse)",
                                        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
                    autoChooser.addOption(
                                        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
                    autoChooser.addOption(
                                        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
                    // Configure the trigger bindings
                    configureBindings();
                    
                    // Configure the LED behaviors
                    configureLEDTriggers();

                    // Forward ports for Limelight
                    PortForwarder.add(5800, "172.29.0.1", 5800);
                    PortForwarder.add(5801, "172.29.0.1", 5801);
                    PortForwarder.add(5802, "172.29.0.1", 5802);
                    PortForwarder.add(5803, "172.29.0.1", 5803);
                    PortForwarder.add(5804, "172.29.0.1", 5804);
                    PortForwarder.add(5805, "172.29.0.1", 5805);
                    PortForwarder.add(5806, "172.29.0.1", 5806);
                    PortForwarder.add(5807, "172.29.0.1", 5807);
                    PortForwarder.add(5808, "172.29.0.1", 5808);
                    PortForwarder.add(5809, "172.29.0.1", 5809);

          }

    /** Maximum reliable shot distance — matches the upper bound of the shooter RPM lookup table. */
    private static final double MAX_SHOT_DISTANCE_METERS = 5.0;

    private void configureLEDTriggers() {
        // 1. DEFAULT STATE: If nothing else is happening, show the match mode.
        m_leds.setDefaultCommand(Commands.run(() -> {
            if (DriverStation.isDisabled()) {
                m_leds.setState(LEDState.IDLE);
            } else if (DriverStation.isAutonomous()) {
                m_leds.setState(LEDState.AUTO);
            } else {
                m_leds.setState(LEDState.TELEOP);
            }
        }, m_leds).ignoringDisable(true));
/*
        // 2. END GAME: Last 30 seconds of Teleop
        new Trigger(() -> DriverStation.isTeleopEnabled() && DriverStation.getMatchTime() > 0 && DriverStation.getMatchTime() <= 30)
            .whileTrue(Commands.run(() -> m_leds.setState(LEDState.END_GAME), m_leds));
*/
        // 3. LOW BATTERY: Battery dips below 11.0V (ignoring disable so it flashes in the pits if battery is dead)
        new Trigger(() -> RobotController.getBatteryVoltage() < 11.0)
            .whileTrue(Commands.run(() -> m_leds.setState(LEDState.LOW_BATTERY), m_leds).ignoringDisable(true));
/* 
        // 4. TOO FAR: Robot is outside reliable shooting range → orange blink
        new Trigger(() -> DriverStation.isTeleopEnabled()
                && drive.distanceToTarget().in(Meters) > MAX_SHOT_DISTANCE_METERS)
            .whileTrue(Commands.run(() -> m_leds.setState(LEDState.TOO_FAR), m_leds));
 */
        // 5. AIM LOCKED: Robot is aimed at goal within 8 degrees (overrides TOO_FAR when aimed)
        new Trigger(() -> drive.isAimedAtTarget(8.0))
            .whileTrue(Commands.run(() -> m_leds.setState(LEDState.AIM_LOCKED), m_leds));

        // 6. SHOOTING INDICATOR: Green when at target speed, Red when spinning up or dropped below
        new Trigger(() -> m_shooter.getCurrentCommand() != null && m_shooter.getCurrentCommand().getName().equals("Shooting"))
            .whileTrue(Commands.run(() -> {
                if (m_shooter.isFlywheelSpinning.getAsBoolean()) {
                    m_leds.setState(LEDState.READY_TO_SHOOT); // Solid Green
                } else {
                    m_leds.setState(LEDState.FAULT); // Blinks Red
                }
            }, m_leds));

        // AprilTag indicator: pixels 0-2 (far left) and 74-76 (far right) → green when tag visible
        new Trigger(vision::hasAprilTagVisible)
            .onTrue(Commands.runOnce(() -> m_leds.setAprilTagVisible(true), m_leds))
            .onFalse(Commands.runOnce(() -> m_leds.setAprilTagVisible(false), m_leds));
            
        // Note: Because WPILib command scheduling is "last scheduled wins", triggers defined lower down 
        // will override triggers defined above them if both conditions are true.
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        // Default command, normal field-relative drive at cruise speed
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        () -> -driveCtrlr.getLeftY()  * getDriveSpeedMultiplier(),
                        () -> -driveCtrlr.getLeftX()  * getDriveSpeedMultiplier(),
                        () -> -driveCtrlr.getRightX() * getDriveSpeedMultiplier()));

        // Keep the shooter flywheel idling when no other commands are using it
        m_shooter.setDefaultCommand(m_shooter.idleCommand());

        // B: Lock facing alliance station (0° alliance-relative) while held
        driveCtrlr
                .b()
                .whileTrue(
                        DriveCommands.joystickDriveAtAngle(
                                drive,
                                () -> -driveCtrlr.getLeftY(),
                                () -> -driveCtrlr.getLeftX(),
                                () -> DriveCommands.isFlipped() ? Rotation2d.kZero
                                        : Rotation2d.k180deg));

        // A: Snap to -45° for crossing angled field hump
        driveCtrlr
                .a()
                .whileTrue(
                        DriveCommands.joystickDriveAtAngle(
                                drive,
                                () -> -driveCtrlr.getLeftY(),
                                () -> -driveCtrlr.getLeftX(),
                                () -> Rotation2d.fromDegrees(DriveCommands.isFlipped() ? 135 : 315)));

        // Y: Snap to +45° for crossing angled field hump
        driveCtrlr
                .y()
                .whileTrue(
                        DriveCommands.joystickDriveAtAngle(
                                drive,
                                () -> -driveCtrlr.getLeftY(),
                                () -> -driveCtrlr.getLeftX(),
                                () -> Rotation2d.fromDegrees(DriveCommands.isFlipped() ? 225 : 45)));

        driveCtrlr.rightStick()
                .whileTrue(DriveCommands.joystickDriveSnake(
                        drive,
                        () -> -driveCtrlr.getLeftY(),
                        () -> -driveCtrlr.getLeftX()));

        // Switch to X pattern when X button is pressed
        driveCtrlr.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

        // RT: Auto-aim to goal
        driveCtrlr.rightTrigger(OIConstants.kTriggerButtonThreshold)
                .whileTrue(DriveCommands.joystickDriveAim(drive,
                        () -> -driveCtrlr.getLeftY(),
                        () -> -driveCtrlr.getLeftX()));

        // Two different ways to do the same thing:

        // Reset gyro to 0° when Back button is pressed
        driveCtrlr
                .back()
                .onTrue(
                        Commands.runOnce(drive::resetHeadingToAlliance, drive)
                                .ignoringDisable(true));

        // Start Button -> Zero swerve heading
        driveCtrlr.start().onTrue(drive.zeroHeadingCommand());

        // ── Operator Controller ──────────────────────────────────────────────────

        // D-Pad Up/Down: Manual feeder forward / reverse (no flywheel)
        operatorCtrlr.povUp().whileTrue(m_shooter.runFeederOnlyCommand());
        operatorCtrlr.povDown().whileTrue(m_shooter.runFeederReverseCommand());

        // D-Pad Left/Right: Manual conveyor forward / reverse
        operatorCtrlr.povLeft().whileTrue(m_conveyor.runConveyorCommand());
        operatorCtrlr.povRight().whileTrue(m_conveyor.runConveyorReverseCommand());

        // RT: Intake forward at trigger-proportional speed (roller only, no conveyor)
        operatorCtrlr
                .rightTrigger(OIConstants.kTriggerButtonThreshold)
                .whileTrue(m_intake.runIntakeOnlyCommand(operatorCtrlr::getRightTriggerAxis));

        // LT: Intake reverse at trigger-proportional speed
        operatorCtrlr
                .leftTrigger(OIConstants.kTriggerButtonThreshold)
                .whileTrue(m_intake.runIntakeOnlyCommand(() -> -operatorCtrlr.getLeftTriggerAxis())
                        .alongWith(m_conveyor.runConveyorReverseCommand()));

        // RS: Agitate intake (oscillate pivot + run roller to jostle balls)
        operatorCtrlr.rightStick().whileTrue(m_intake.agitateCommand());

        // RB: Deploy intake arm to deployed position
        operatorCtrlr.rightBumper().onTrue(m_intake.deployIntakeCommand());

        // LB: Retract intake arm to retracted position
        operatorCtrlr.leftBumper().onTrue(m_intake.retractIntakeCommand());

        // Left Stick Y: Fine-tune arm position (deadbanded; push forward = toward deployed)
        operatorCtrlr.axisGreaterThan(1, OIConstants.kDriveDeadband)
                .or(operatorCtrlr.axisLessThan(1, -OIConstants.kDriveDeadband))
                .whileTrue(m_intake.nudgePivotCommand(() -> operatorCtrlr.getLeftY()));

        // Y Button: Full shoot sequence (spin up → feeder + conveyor when at speed)
        operatorCtrlr.y().toggleOnTrue(
                m_shooter.runShooterCommand()
                        .alongWith(
                                Commands.waitUntil(m_shooter.isFlywheelSpinning)
                                        .andThen(m_conveyor.runConveyorCommand())));

        // A Button: Toggle auto/manual shooter speed mode
        operatorCtrlr.a().onTrue(m_shooter.toggleAutoSpeedCommand());

        // X/B Buttons: Adjust shooter target speed -/+ 100 RPM
        operatorCtrlr.x().onTrue(m_shooter.adjustShooterSpeedCommand(-100));
        operatorCtrlr.b().onTrue(m_shooter.adjustShooterSpeedCommand(+100));

        // Start Button: Toggle shooter sleep mode (completely stop overriding idle)
        operatorCtrlr.start().toggleOnTrue(m_shooter.stopShooterCommand());

                // D-pad snap to heading (alliance-aware: Up = away from your station)
        driveCtrlr.povUp().whileTrue(DriveCommands.joystickDriveAtAngle(
                        drive,
                        () -> -driveCtrlr.getLeftY(),
                        () -> -driveCtrlr.getLeftX(),
                        () -> Rotation2d.fromDegrees(DriveCommands.isFlipped() ? 180 : 0)));

        driveCtrlr.povRight().whileTrue(DriveCommands.joystickDriveAtAngle(
                        drive,
                        () -> -driveCtrlr.getLeftY(),
                        () -> -driveCtrlr.getLeftX(),
                        () -> Rotation2d.fromDegrees(DriveCommands.isFlipped() ? 270 : 90)));

        driveCtrlr.povDown().whileTrue(DriveCommands.joystickDriveAtAngle(
                        drive,
                        () -> -driveCtrlr.getLeftY(),
                        () -> -driveCtrlr.getLeftX(),
                        () -> Rotation2d.fromDegrees(DriveCommands.isFlipped() ? 0 : 180)));

        driveCtrlr.povLeft().whileTrue(DriveCommands.joystickDriveAtAngle(
                        drive,
                        () -> -driveCtrlr.getLeftY(),
                        () -> -driveCtrlr.getLeftX(),
                        () -> Rotation2d.fromDegrees(DriveCommands.isFlipped() ? 90 : 270)));

        drive.inAllianceZoneTrigger().onTrue(DriveCommands.setGoalTargetCommand());
    }

    /** Returns drive speed scalar: LB+RB = super slow, LB = slow, RB = burst, default = cruise. */
    private double getDriveSpeedMultiplier() {
        boolean lb = driveCtrlr.leftBumper().getAsBoolean();
        boolean rb = driveCtrlr.rightBumper().getAsBoolean();
        if (lb && rb) return OIConstants.kSuperSlowSpeedMultiplier;
        if (lb)       return OIConstants.kSlowSpeedMultiplier;
        if (rb)       return OIConstants.kFastSpeedMultiplier;
        return OIConstants.kNormalSpeedMultiplier;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public static RobotContainer getInstance() {
        return instance;
    }

    public void onTeleopInit() {
            m_leds.setState(LEDState.TELEOP);
            drive.resetHeadingToAlliance();
    }

    public void onAutonomousInit() {
            m_leds.setState(LEDState.AUTO);
    }

    public void onDisabledInit() {
            m_leds.setState(LEDState.IDLE);
    }
}
