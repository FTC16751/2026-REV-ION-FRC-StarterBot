// package frc.robot.commands;

// import static org.junit.jupiter.api.Assertions.assertTrue;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import frc.robot.subsystems.drive.Drive;
// import frc.robot.subsystems.drive.GyroIO;
// import frc.robot.subsystems.drive.ModuleIO;
// import org.junit.jupiter.api.Test;

// /** Unit tests for {@link DriveCommands}. */
// public class DriveCommandsTest {

//   // Lightweight test Drive that captures the last ChassisSpeeds passed to runVelocity
//   private static class TestDrive extends Drive {
//     private ChassisSpeeds lastSpeedsField = new ChassisSpeeds();
//     private Rotation2d rotation = Rotation2d.kZero;

//     public TestDrive() {
//       // Provide minimal dummy IO implementations; Drive constructor needs these.
//       super(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
//     }

//     public void setRotation(Rotation2d rotation) {
//       this.rotation = rotation;
//     }

//     @Override
//     public Rotation2d getRotation() {
//       return rotation;
//     }

//     @Override
//     public void runVelocity(ChassisSpeeds speeds) {
//       // Convert robot-relative speeds back to field-relative for assertions:
//       double theta = rotation.getRadians();
//       double vx_r = speeds.vxMetersPerSecond;
//       double vy_r = speeds.vyMetersPerSecond;
//       double vx_f = vx_r * Math.cos(theta) - vy_r * Math.sin(theta);
//       double vy_f = vx_r * Math.sin(theta) + vy_r * Math.cos(theta);
//       lastSpeedsField = new ChassisSpeeds(vx_f, vy_f, speeds.omegaRadiansPerSecond);
//     }

//     public ChassisSpeeds getLastSpeedsField() {
//       return lastSpeedsField;
//     }

//     @Override
//     public double getMaxLinearSpeedMetersPerSec() {
//       // Use a known max speed to make assertions straightforward
//       return 1.0;
//     }
//   }

//   @Test
//   public void forwardJoystickProducesPositiveVx() {
//     TestDrive drive = new TestDrive();

//     // Simulate left joystick pushed forward: getLeftY() returns negative
//     double leftY = -0.6; // joystick API returns negative when pushed forward
//     double leftX = 0.0;
//     double rightX = 0.0;

//     // RobotContainer config in this project passes suppliers as: () -> -getLeftY(), () -> -getLeftX()
//     var cmd = DriveCommands.joystickDrive(drive, () -> -leftY, () -> -leftX, () -> -rightX);

//     // Run the command once
//     cmd.initialize();
//     cmd.execute();
//     cmd.end(false);

//     ChassisSpeeds fieldSpeeds = drive.getLastSpeedsField();

//     // Expect positive vx when joystick pushed forward (leftY negative)
//     assertTrue(fieldSpeeds.vxMetersPerSecond > 0.0, "Expected positive field vx when joystick pushed forward");
//   }

//   @Test
//   public void leftJoystickProducesPositiveVy() {
//     TestDrive drive = new TestDrive();

//     // Simulate left joystick pushed left: getLeftX() returns negative
//     double leftY = 0.0;
//     double leftX = -0.5; // joystick API returns negative when pushed left
//     double rightX = 0.0;

//     // RobotContainer config in this project passes suppliers as: () -> -getLeftY(), () -> -getLeftX()
//     var cmd = DriveCommands.joystickDrive(drive, () -> -leftY, () -> -leftX, () -> -rightX);

//     // Run the command once
//     cmd.initialize();
//     cmd.execute();
//     cmd.end(false);

//     ChassisSpeeds fieldSpeeds = drive.getLastSpeedsField();

//     // Expect positive vy when joystick pushed left (leftX negative)
//     assertTrue(fieldSpeeds.vyMetersPerSecond > 0.0, "Expected positive field vy when joystick pushed left");
//   }

//   @Test
//   public void forwardAndLeftJoystickProducePositiveFieldSpeedsAtArbitraryRotation() {
//     TestDrive drive = new TestDrive();

//     // Choose an arbitrary rotation (e.g., 1.234 rad)
//     drive.setRotation(new Rotation2d(1.234));

//   // Forward input
//   final double fwdLeftY = -0.7;
//   final double fwdLeftX = 0.0;
//   final double fwdRightX = 0.0;
//   var cmdFwd = DriveCommands.joystickDrive(drive, () -> -fwdLeftY, () -> -fwdLeftX, () -> -fwdRightX);
//     cmdFwd.initialize();
//     cmdFwd.execute();
//     cmdFwd.end(false);
//     ChassisSpeeds fieldFwd = drive.getLastSpeedsField();
//     assertTrue(fieldFwd.vxMetersPerSecond > 0.0, "Expected positive field vx at arbitrary rotation when pushing forward");

//   // Left input
//   final double leftLeftY = 0.0;
//   final double leftLeftX = -0.6;
//   final double leftRightX = 0.0;
//   var cmdLeft = DriveCommands.joystickDrive(drive, () -> -leftLeftY, () -> -leftLeftX, () -> -leftRightX);
//     cmdLeft.initialize();
//     cmdLeft.execute();
//     cmdLeft.end(false);
//     ChassisSpeeds fieldLeft = drive.getLastSpeedsField();
//     assertTrue(fieldLeft.vyMetersPerSecond > 0.0, "Expected positive field vy at arbitrary rotation when pushing left");
//   }
// }
