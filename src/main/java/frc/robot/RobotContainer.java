// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveCommands;
//import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.intake.Intake;
//import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;

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
  private static RobotContainer instance;

    // The driver's controller
    private final CommandXboxController driveCtrlr = new CommandXboxController(OIConstants.kDriverControllerPort);

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
        m_shooter = new Shooter(new ShooterIOSpark());
        m_intake = new Intake(new frc.robot.subsystems.intake.IntakeIOSpark());
                vision = new Vision(
                        drive::addVisionMeasurement,
                        new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation));
                break;

      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        m_shooter = new Shooter(new ShooterIOSim());
        m_intake = new Intake(new frc.robot.subsystems.intake.IntakeIOSim());
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});
        break;

      default:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        m_shooter = new Shooter(new ShooterIO() {});
        m_intake = new Intake(new frc.robot.subsystems.intake.IntakeIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});
        break;
    }

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption(
                "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
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
        // Default command, normal field-relative drive
        drive.setDefaultCommand(
                DriveCommands.joystickDriveAbsoluteAngle(
                        drive,
                        () -> -driveCtrlr.getLeftY(),
                        () -> -driveCtrlr.getLeftX(),
                        () -> -driveCtrlr.getRightY(),
                        () -> -driveCtrlr.getRightX()));

        // Lock to 0° Alliance Relative when A button is held
        driveCtrlr
                .a()
                .whileTrue(
                        DriveCommands.joystickDriveAtAngle(
                                drive,
                                () -> -driveCtrlr.getLeftY(),
                                () -> -driveCtrlr.getLeftX(),
                                () -> DriveCommands.isFlipped() ?
                                Rotation2d.kZero
                                : Rotation2d.k180deg
                                ));

        driveCtrlr.rightStick()
            .whileTrue(DriveCommands.joystickDriveSnake(
                                drive,
                                () -> -driveCtrlr.getLeftY(),
                                () -> -driveCtrlr.getLeftX()));

        // Switch to X pattern when X button is pressed
        driveCtrlr.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

        driveCtrlr.rightBumper()
                .whileTrue(DriveCommands.joystickDriveAim(drive,
                        () -> -driveCtrlr.getLeftY(),
                        () -> -driveCtrlr.getLeftX()));

        // Two different ways to do the same thing:

        // Reset gyro to 0° when B button is pressed
        driveCtrlr
                .b()
                .onTrue(
                        Commands.runOnce(
                                () -> drive.setPose(
                                        new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                                drive)
                                .ignoringDisable(true));

        // Start Button -> Zero swerve heading
        driveCtrlr.start().onTrue(drive.zeroHeadingCommand());

        // TODO: extend/retract intake. Possibly as a function of the right trigger
        // position?
        // Right Trigger -> Run fuel intake in reverse
        driveCtrlr
                .rightTrigger(OIConstants.kTriggerButtonThreshold)
                .whileTrue(m_intake.runIntakeCommand());

        // Left Trigger -> Run fuel intake in reverse
        driveCtrlr
                .leftTrigger(OIConstants.kTriggerButtonThreshold)
                .whileTrue(m_intake.runExtakeCommand());

    // Y Button -> Spin up flywheel, then run feeder + conveyor once at speed
    driveCtrlr.y().toggleOnTrue(
        m_shooter.runShooterCommand()
            .alongWith(
                Commands.waitUntil(m_shooter.isFlywheelSpinning)
                    .andThen(m_intake.runIntakeCommand())));

    // D-pad snap to heading (field-relative)
    driveCtrlr.povUp().whileTrue(DriveCommands.joystickDriveAtAngle(
        drive,
        () -> -driveCtrlr.getLeftY(),
        () -> -driveCtrlr.getLeftX(),
        () -> Rotation2d.fromDegrees(0)));

    driveCtrlr.povRight().whileTrue(DriveCommands.joystickDriveAtAngle(
        drive,
        () -> -driveCtrlr.getLeftY(),
        () -> -driveCtrlr.getLeftX(),
        () -> Rotation2d.fromDegrees(-90)));

    driveCtrlr.povDown().whileTrue(DriveCommands.joystickDriveAtAngle(
        drive,
        () -> -driveCtrlr.getLeftY(),
        () -> -driveCtrlr.getLeftX(),
        () -> Rotation2d.fromDegrees(180)));

    driveCtrlr.povLeft().whileTrue(DriveCommands.joystickDriveAtAngle(
        drive,
        () -> -driveCtrlr.getLeftY(),
        () -> -driveCtrlr.getLeftX(),
        () -> Rotation2d.fromDegrees(90)));

        drive.inAllianceZoneTrigger().onTrue(DriveCommands.setGoalTargetCommand());
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
}
