package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

/**
 * ACTIVE SINGLE-MOTOR LIFT SUBSYSTEM
 *
 * A subsystem owns the behavior of one physical mechanism. This class controls
 * the active "leftLift" motor that RobotHardware maps from the Control Hub.
 *
 * PHYSICAL ROBOT CONNECTION:
 * The motor turns the lift mechanism. Its built-in/external encoder reports a
 * position in ticks, allowing the software to command repeatable heights.
 *
 * IMPORTANT:
 * The encoder heights in RobotConstants are starting values. They must be
 * measured and adjusted on the real robot before competition use.
 */
public final class LiftSubsystem {
    private final DcMotorEx liftMotor;
    private final TouchSensor magneticLimitSwitch;

    private boolean homed;
    private boolean previousLimitPressed;

    /**
     * Reuse the motor already mapped by RobotHardware instead of looking it up
     * again. This keeps hardware ownership consistent across the project.
     */
    public LiftSubsystem(RobotHardware robot) {
        liftMotor = robot.leftLift;
        magneticLimitSwitch = robot.magneticLimitSwitch;

        // BRAKE resists free movement when motor power is zero. This is commonly
        // useful for lifts, although the real mechanism still determines whether
        // additional holding power/gravity compensation is needed.
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Preserve encoder position and use it for closed-loop preset movement.
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // If the lift starts at the bottom, establish a valid height reference
        // immediately. Otherwise the lift must be lowered until the switch is hit.
        previousLimitPressed = isHomeLimitPressed();
        if (previousLimitPressed) {
            zeroEncoderAtHome();
        }
    }

    /**
     * Poll the home switch once per OpMode/Action loop. On the rising edge of
     * the switch, stop the lift and define that physical position as zero ticks.
     */
    public void update() {
        boolean limitPressed = isHomeLimitPressed();

        if (limitPressed && !previousLimitPressed) {
            zeroEncoderAtHome();
        }

        previousLimitPressed = limitPressed;
    }

    private void zeroEncoderAtHome() {
        liftMotor.setPower(0.0);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setTargetPosition(RobotConstants.Lift.HOME_TICKS);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        homed = true;
    }

    /**
     * Begin moving toward an encoder target without blocking the OpMode thread.
     * The target is clamped to the software limits defined in RobotConstants.
     */
    public void moveTo(int requestedTicks) {
        update();

        int safeTarget = Math.max(
                RobotConstants.Lift.HOME_TICKS,
                Math.min(RobotConstants.Lift.MAX_TICKS, requestedTicks)
        );

        // Never command farther downward when the physical home switch is active.
        if (isHomeLimitPressed() && safeTarget <= getHeightTicks()) {
            stop();
            return;
        }

        liftMotor.setTargetPosition(safeTarget);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(RobotConstants.Lift.MOVE_POWER);
    }

    /** Return the lift to the bottom/base preset. */
    public void moveHome() {
        moveTo(RobotConstants.Lift.HOME_TICKS);
    }

    /** Raise the lift to the current highest preset. */
    public void moveHigh() {
        moveTo(RobotConstants.Lift.HIGH_TICKS);
    }

    /** Current encoder position, useful for telemetry and autonomous completion. */
    public int getCurrentPosition() {
        return liftMotor.getCurrentPosition();
    }

    /**
     * Lift height above the magnetic-switch home position, in encoder ticks.
     * A physical height in inches requires a measured ticks-per-inch constant.
     */
    public int getHeightTicks() {
        return Math.max(RobotConstants.Lift.HOME_TICKS, getCurrentPosition());
    }

    /** Height expressed as a 0.0-to-1.0 fraction of the configured travel. */
    public double getHeightFraction() {
        return Math.min(1.0, (double) getHeightTicks() / RobotConstants.Lift.MAX_TICKS);
    }

    /** True after the magnetic switch has established a reliable encoder zero. */
    public boolean isHomed() {
        return homed;
    }

    /** True while the magnet is activating the bottom/home switch. */
    public boolean isHomeLimitPressed() {
        return magneticLimitSwitch.isPressed();
    }

    /** Target currently stored in the motor controller. */
    public int getTargetPosition() {
        return liftMotor.getTargetPosition();
    }

    /**
     * Returns true when the lift is close enough to its requested position that
     * an autonomous Action can move on to the next step.
     */
    public boolean atTarget() {
        return Math.abs(getTargetPosition() - getHeightTicks())
                <= RobotConstants.Lift.TOLERANCE_TICKS;
    }

    /** Immediately remove motor power. */
    public void stop() {
        liftMotor.setPower(0.0);
    }

    /** Manually raise the lift while retaining encoder height feedback. */
    public void raise() {
        update();
        if (homed && getHeightTicks() >= RobotConstants.Lift.MAX_TICKS) {
            stop();
            return;
        }

        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setPower(RobotConstants.Lift.MANUAL_POWER);
    }

    /** Manually lower the lift, stopping at the magnetic home switch. */
    public void lower() {
        update();
        if (isHomeLimitPressed()) {
            stop();
            return;
        }

        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setPower(-RobotConstants.Lift.MANUAL_POWER);
    }
}
