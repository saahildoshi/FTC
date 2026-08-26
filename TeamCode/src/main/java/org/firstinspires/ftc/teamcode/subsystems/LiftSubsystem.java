package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

/**
 * ACTIVE SINGLE-MOTOR SWYFT LIFT SUBSYSTEM
 *
 * This class keeps the existing project API and hardware names while replacing
 * RUN_TO_POSITION with a custom PID controller plus gravity feedforward.
 *
 * EXISTING PROJECT NAMES PRESERVED:
 * - Class: LiftSubsystem
 * - Motor: RobotHardware.leftLift
 * - Home sensor: RobotHardware.magneticLimitSwitch
 * - Methods: moveTo(), moveHome(), moveHigh(), update(), raise(), lower(), etc.
 *
 * HOME REFERENCE:
 * The REV magnetic limit switch is the authoritative physical home position.
 * When the switch is reached, the encoder is reset to HOME_TICKS.
 *
 * IMPORTANT:
 * update() must be called every OpMode/Action loop while using a preset position.
 */
public final class LiftSubsystem {
    private final DcMotorEx liftMotor;
    private final TouchSensor magneticLimitSwitch;

    // ----------------------------------------------------------------------
    // CUSTOM PID + GRAVITY FEEDFORWARD
    // ----------------------------------------------------------------------
    // Starting values only. Tune these on the physical SWYFT lift.
    private static final double KP = 0.0030;
    private static final double KI = 0.0000;
    private static final double KD = 0.0001;
    private static final double KG = 0.08;

    // Slow downward power used only while physically searching for home.
    private static final double HOMING_POWER = -0.20;

    // Prevents integral windup if KI is enabled later.
    private static final double INTEGRAL_LIMIT = 500.0;

    private final ElapsedTime pidTimer = new ElapsedTime();

    private double integralSum;
    private double previousError;

    private int targetPosition = RobotConstants.Lift.HOME_TICKS;

    private boolean homed;
    private boolean homing;
    private boolean positionControlActive;
    private boolean previousLimitPressed;

    /**
     * Reuse the motor and magnetic switch already mapped by RobotHardware.
     */
    public LiftSubsystem(RobotHardware robot) {
        liftMotor = robot.leftLift;
        magneticLimitSwitch = robot.magneticLimitSwitch;

        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*
         * The subsystem now performs its own PID calculations. RUN_WITHOUT_ENCODER
         * still allows getCurrentPosition() to read the motor encoder; it simply
         * prevents the hub's built-in RUN_TO_POSITION controller from taking over.
         */
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        resetPidState();

        // If the lift already starts on the magnet, immediately establish zero.
        previousLimitPressed = isHomeLimitPressed();
        if (previousLimitPressed) {
            zeroEncoderAtHome();
        }
    }

    /**
     * Main non-blocking lift update. Call this once every OpMode/Action loop.
     */
    public void update() {
        boolean limitPressed = isHomeLimitPressed();

        // --------------------------------------------------------------
        // HOMING STATE
        // --------------------------------------------------------------
        if (homing) {
            if (limitPressed) {
                zeroEncoderAtHome();
            } else {
                // Search downward slowly until the REV magnetic switch is found.
                liftMotor.setPower(HOMING_POWER);
            }

            previousLimitPressed = limitPressed;
            return;
        }

        // If the switch becomes active during normal operation, that physical
        // position is home regardless of small accumulated encoder error.
        if (limitPressed && !previousLimitPressed) {
            zeroEncoderAtHome();
            previousLimitPressed = true;
            return;
        }

        // --------------------------------------------------------------
        // CUSTOM PID POSITION CONTROL
        // --------------------------------------------------------------
        if (positionControlActive) {
            int currentPosition = getCurrentPosition();
            double error = targetPosition - currentPosition;

            double dt = pidTimer.seconds();
            pidTimer.reset();

            // Avoid divide-by-zero and unusually large timing gaps.
            dt = Range.clip(dt, 0.001, 0.100);

            // Integral term with anti-windup.
            integralSum += error * dt;
            integralSum = Range.clip(
                    integralSum,
                    -INTEGRAL_LIMIT,
                    INTEGRAL_LIMIT
            );

            // Derivative measures how quickly the position error is changing.
            double derivative = (error - previousError) / dt;
            previousError = error;

            double pidOutput =
                    (KP * error)
                            + (KI * integralSum)
                            + (KD * derivative);

            /*
             * Vertical lift gravity compensation.
             * Positive power is assumed to raise the existing leftLift motor,
             * matching the project's existing raise()/lower() convention.
             */
            double gravityFeedforward = 0.0;
            if (targetPosition > RobotConstants.Lift.HOME_TICKS
                    + RobotConstants.Lift.TOLERANCE_TICKS
                    || currentPosition > RobotConstants.Lift.HOME_TICKS
                    + RobotConstants.Lift.TOLERANCE_TICKS) {
                gravityFeedforward = KG;
            }

            double motorPower = pidOutput + gravityFeedforward;

            // Keep preset movement within the project's existing maximum output.
            motorPower = Range.clip(
                    motorPower,
                    -RobotConstants.Lift.MOVE_POWER,
                    RobotConstants.Lift.MOVE_POWER
            );

            // Physical bottom limit: never drive farther down while on the magnet.
            if (limitPressed && motorPower < 0.0) {
                motorPower = 0.0;
            }

            // Software upper limit.
            if (currentPosition >= RobotConstants.Lift.MAX_TICKS
                    && motorPower > 0.0) {
                motorPower = 0.0;
            }

            liftMotor.setPower(motorPower);
        }

        previousLimitPressed = limitPressed;
    }

    /**
     * Define the current magnetic-switch position as physical home/zero.
     */
    private void zeroEncoderAtHome() {
        liftMotor.setPower(0.0);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        targetPosition = RobotConstants.Lift.HOME_TICKS;

        homed = true;
        homing = false;
        positionControlActive = false;

        resetPidState();
    }

    /** Reset accumulated PID history whenever control modes/targets change. */
    private void resetPidState() {
        integralSum = 0.0;
        previousError = 0.0;
        pidTimer.reset();
    }

    /**
     * Begin moving toward an encoder target without blocking the OpMode thread.
     * Existing method name is preserved for the rest of the project.
     */
    public void moveTo(int requestedTicks) {
        int safeTarget = Math.max(
                RobotConstants.Lift.HOME_TICKS,
                Math.min(RobotConstants.Lift.MAX_TICKS, requestedTicks)
        );

        /*
         * HOME is special: use the magnetic switch rather than trusting encoder
         * position alone. This automatically corrects accumulated encoder drift.
         */
        if (safeTarget == RobotConstants.Lift.HOME_TICKS) {
            moveHome();
            return;
        }

        targetPosition = safeTarget;
        homing = false;
        positionControlActive = true;
        resetPidState();
    }

    /**
     * Return to physical home. The lift moves downward until the REV magnetic
     * switch activates, then the encoder is reset to HOME_TICKS.
     */
    public void moveHome() {
        targetPosition = RobotConstants.Lift.HOME_TICKS;
        positionControlActive = false;
        resetPidState();

        if (isHomeLimitPressed()) {
            zeroEncoderAtHome();
            previousLimitPressed = true;
            return;
        }

        homing = true;
    }

    /** Raise the lift to the project's existing highest preset. */
    public void moveHigh() {
        moveTo(RobotConstants.Lift.HIGH_TICKS);
    }

    /** Current encoder position, useful for telemetry and autonomous completion. */
    public int getCurrentPosition() {
        return liftMotor.getCurrentPosition();
    }

    /**
     * Lift height above the magnetic-switch home position, in encoder ticks.
     */
    public int getHeightTicks() {
        return Math.max(RobotConstants.Lift.HOME_TICKS, getCurrentPosition());
    }

    /** Height expressed as a 0.0-to-1.0 fraction of configured travel. */
    public double getHeightFraction() {
        return Math.min(
                1.0,
                (double) getHeightTicks() / RobotConstants.Lift.MAX_TICKS
        );
    }

    /** True after the magnetic switch has established a reliable encoder zero. */
    public boolean isHomed() {
        return homed;
    }

    /** True while the magnet is activating the bottom/home switch. */
    public boolean isHomeLimitPressed() {
        return magneticLimitSwitch.isPressed();
    }

    /**
     * Existing getter name preserved. The custom controller stores its own target
     * because RUN_WITHOUT_ENCODER does not use the motor controller's target field.
     */
    public int getTargetPosition() {
        return targetPosition;
    }

    /**
     * True when the active command has completed within the configured tolerance.
     */
    public boolean atTarget() {
        // A home command is complete only after the physical switch establishes zero.
        if (targetPosition == RobotConstants.Lift.HOME_TICKS) {
            return homed && isHomeLimitPressed() && !homing;
        }

        if (homing) {
            return false;
        }

        return Math.abs(targetPosition - getHeightTicks())
                <= RobotConstants.Lift.TOLERANCE_TICKS;
    }

    /** Immediately remove motor power and cancel active automatic movement. */
    public void stop() {
        homing = false;
        positionControlActive = false;
        liftMotor.setPower(0.0);
        resetPidState();
    }

    /** Manually raise the lift while retaining encoder height feedback. */
    public void raise() {
        homing = false;
        positionControlActive = false;
        resetPidState();

        if (homed && getHeightTicks() >= RobotConstants.Lift.MAX_TICKS) {
            liftMotor.setPower(0.0);
            return;
        }

        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setPower(RobotConstants.Lift.MANUAL_POWER);
    }

    /** Manually lower the lift, stopping and zeroing at the magnetic home switch. */
    public void lower() {
        homing = false;
        positionControlActive = false;
        resetPidState();

        if (isHomeLimitPressed()) {
            zeroEncoderAtHome();
            previousLimitPressed = true;
            return;
        }

        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setPower(-RobotConstants.Lift.MANUAL_POWER);
    }
}
