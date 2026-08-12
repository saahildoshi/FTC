package org.firstinspires.ftc.teamcode.teleop;

/**
 * OPTIONAL TELEOP CONTROL EXAMPLES
 *
 * This class intentionally contains no active OpMode. Copy only the controls
 * that match the mechanisms the team ultimately builds into TeleOpDrive.java.
 */
public final class MechanismControlOptions {
    private MechanismControlOptions() { }

    /*
     * Example declarations after choosing mechanisms:
     *
     * MechanismOptions.Lift lift = new MechanismOptions.Lift(hardwareMap);
     * MechanismOptions.Arm arm = new MechanismOptions.Arm(hardwareMap);
     * MechanismOptions.Intake intake = new MechanismOptions.Intake(hardwareMap);
     * MechanismOptions.Claw claw = new MechanismOptions.Claw(hardwareMap);
     *
     * Suggested Gamepad 2 layout to evaluate:
     *
     * left_stick_y     -> manual lift
     * right_stick_y    -> manual arm
     * dpad_down        -> lift home preset
     * dpad_left        -> lift low preset
     * dpad_right       -> lift mid preset
     * dpad_up          -> lift high preset
     * right_trigger    -> intake
     * left_trigger     -> outtake
     * right_bumper     -> close claw
     * left_bumper      -> open claw
     *
     * Example loop logic (still intentionally commented):
     *
     * double liftPower = -gamepad2.left_stick_y;
     * if (Math.abs(liftPower) > 0.08) {
     *     lift.manual(liftPower * 0.6);
     * } else {
     *     lift.stop();
     * }
     *
     * double armPower = -gamepad2.right_stick_y;
     * if (Math.abs(armPower) > 0.08) {
     *     arm.manual(armPower * 0.45);
     * } else {
     *     arm.stop();
     * }
     *
     * if (gamepad2.dpad_down) lift.moveTo(MechanismOptions.Lift.HOME_TICKS);
     * if (gamepad2.dpad_left) lift.moveTo(MechanismOptions.Lift.LOW_TICKS);
     * if (gamepad2.dpad_right) lift.moveTo(MechanismOptions.Lift.MID_TICKS);
     * if (gamepad2.dpad_up) lift.moveTo(MechanismOptions.Lift.HIGH_TICKS);
     *
     * if (gamepad2.right_trigger > 0.1) {
     *     intake.intake();
     * } else if (gamepad2.left_trigger > 0.1) {
     *     intake.outtake();
     * } else {
     *     intake.stop();
     * }
     *
     * if (gamepad2.left_bumper) claw.open();
     * if (gamepad2.right_bumper) claw.close();
     */
}
