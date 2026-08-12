package org.firstinspires.ftc.teamcode.teleop;

/**
 * MODULAR TELEOP INTEGRATION GUIDE
 *
 * The repository's existing TeleOpDrive contains tuned driver behavior and is
 * intentionally left as the competition/reference implementation.
 *
 * This file documents how that behavior can be migrated into the modular
 * architecture without forcing a risky rewrite all at once.
 *
 * ROBOT DATA FLOW:
 *
 * Gamepad
 *   -> TeleOp OpMode (driver decisions)
 *      -> DriveSubsystem (chassis commands)
 *         -> existing Road Runner MecanumDrive
 *            -> four physical mecanum motors
 *
 * Gamepad 2 (future)
 *   -> TeleOp OpMode
 *      -> Lift / Arm / Intake / Claw subsystems
 *         -> physical mechanism motors and servos
 *
 * A future active modular TeleOp can follow this pattern:
 *
 * --------------------------------------------------------------------------
 *
 * @TeleOp(name = "Modular TeleOp")
 * public class ModularTeleOp extends OpMode {
 *     private DriveSubsystem drive;
 *     private RobotTelemetry robotTelemetry;
 *     private Button fieldCentricToggle = new Button();
 *     private boolean fieldCentric = true;
 *
 *     public void init() {
 *         // Pose2d(0,0,0) means the software initially calls the robot's current
 *         // location field X=0, field Y=0, heading=0.
 *         drive = new DriveSubsystem(hardwareMap, new Pose2d(0, 0, 0));
 *         robotTelemetry = new RobotTelemetry(telemetry);
 *     }
 *
 *     public void loop() {
 *         // FTC joystick Y is positive downward, so negate it for forward.
 *         double forward = -gamepad1.left_stick_y;
 *         double strafe = -gamepad1.left_stick_x;
 *         double turn = -gamepad1.right_stick_x;
 *
 *         // Example one-press toggle rather than toggling every 20 ms while held.
 *         if (fieldCentricToggle.wasPressed(gamepad1.y)) {
 *             fieldCentric = !fieldCentric;
 *         }
 *
 *         if (fieldCentric) {
 *             drive.fieldCentric(forward, strafe, turn);
 *         } else {
 *             drive.robotCentric(forward, strafe, turn);
 *         }
 *
 *         // Future mechanism calls belong here after the hardware is selected.
 *         // lift.manual(-gamepad2.left_stick_y);
 *         // arm.manual(-gamepad2.right_stick_y);
 *         // intake.intake();
 *         // claw.open();
 *
 *         robotTelemetry.addDriveData(drive);
 *         telemetry.addData("Field Centric", fieldCentric);
 *         robotTelemetry.update();
 *     }
 *
 *     public void stop() {
 *         drive.stop();
 *     }
 * }
 *
 * --------------------------------------------------------------------------
 *
 * WHY THIS FILE IS COMMENTED INSTEAD OF ACTIVE:
 * The current TeleOpDrive includes heading hold, acceleration limiting, return
 * to origin, slow mode, and other tuned behavior. Replacing it immediately with
 * a simpler example would remove working features. Students can migrate one
 * feature at a time and test each change on the robot.
 */
public final class ModularTeleOpExample {
    private ModularTeleOpExample() { }
}
