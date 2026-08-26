package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.util.CustomPIDController;

/**
 * ACTIVE SINGLE-MOTOR SWYFT LIFT SUBSYSTEM
 *
 * This version uses the logic from the original SwyftLift.java implementation:
 * custom PID, gravity feedforward, software limits, and preset positions.
 *
 * The existing project class/method names are preserved so current code can keep
 * using LiftSubsystem, moveTo(), moveHome(), moveHigh(), update(), etc.
 */
public final class LiftSubsystem {

    private final DcMotorEx liftMotor;

    // ---------------------------------
    // PID CONSTANTS
    // These WILL need to be tuned
    // ---------------------------------

    private double kP = 0.003;
    private double kI = 0.0;
    private double kD = 0.0001;

    // Gravity compensation
    private double kG = 0.08;

    private final CustomPIDController pid;

    private int targetPosition = RobotConstants.Lift.HOME_TICKS;

    public LiftSubsystem(RobotHardware robot) {

        liftMotor = robot.leftLift;

        // Change if your lift runs backwards
        liftMotor.setDirection(DcMotor.Direction.FORWARD);

        liftMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        /*
         * We are doing our OWN PID,
         * so we do NOT use RUN_TO_POSITION.
         */
        liftMotor.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );

        liftMotor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        pid = new CustomPIDController(
                kP,
                kI,
                kD
        );

        pid.setTarget(RobotConstants.Lift.HOME_TICKS);
    }

    /**
     * MUST be called every OpMode loop.
     */
    public void update() {

        int currentPosition =
                liftMotor.getCurrentPosition();

        // ------------------------------
        // PID
        // ------------------------------

        double pidOutput =
                pid.calculate(currentPosition);

        // ------------------------------
        // GRAVITY FEEDFORWARD
        // ------------------------------

        double gravityFeedforward = 0;

        /*
         * Do not fight against the physical
         * bottom of the slide.
         */
        if (targetPosition > 50 ||
                currentPosition > 50) {

            gravityFeedforward = kG;
        }

        // ------------------------------
        // FINAL MOTOR POWER
        // ------------------------------

        double motorPower =
                pidOutput + gravityFeedforward;

        motorPower = Range.clip(
                motorPower,
                -1.0,
                1.0
        );

        // ------------------------------
        // SOFTWARE LIMITS
        // ------------------------------

        if (currentPosition <= RobotConstants.Lift.HOME_TICKS &&
                motorPower < 0) {

            motorPower = 0;
        }

        if (currentPosition >= RobotConstants.Lift.MAX_TICKS &&
                motorPower > 0) {

            motorPower = 0;
        }

        liftMotor.setPower(motorPower);
    }

    public void moveTo(int target) {

        targetPosition = Range.clip(
                target,
                RobotConstants.Lift.HOME_TICKS,
                RobotConstants.Lift.MAX_TICKS
        );

        pid.setTarget(targetPosition);
    }

    /** Return the lift to the bottom/base preset. */
    public void moveHome() {
        moveTo(RobotConstants.Lift.HOME_TICKS);
    }

    /** Raise the lift to the current highest preset. */
    public void moveHigh() {
        moveTo(RobotConstants.Lift.HIGH_TICKS);
    }

    public int getTargetPosition() {
        return targetPosition;
    }

    public int getCurrentPosition() {
        return liftMotor.getCurrentPosition();
    }

    public int getHeightTicks() {
        return Math.max(
                RobotConstants.Lift.HOME_TICKS,
                getCurrentPosition()
        );
    }

    public double getHeightFraction() {
        return Math.min(
                1.0,
                (double) getHeightTicks() / RobotConstants.Lift.MAX_TICKS
        );
    }

    public double getError() {

        return targetPosition -
                liftMotor.getCurrentPosition();
    }

    public boolean atTarget() {

        return Math.abs(getError()) <
                RobotConstants.Lift.TOLERANCE_TICKS;
    }

    public void resetEncoder() {

        liftMotor.setPower(0);

        liftMotor.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );

        liftMotor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        targetPosition = RobotConstants.Lift.HOME_TICKS;

        pid.reset();
        pid.setTarget(RobotConstants.Lift.HOME_TICKS);
    }

    /**
     * Compatibility method for existing telemetry/code.
     * This original SwyftLift version does not use magnetic-switch homing.
     */
    public boolean isHomed() {
        return getCurrentPosition() == RobotConstants.Lift.HOME_TICKS;
    }

    /**
     * Compatibility method for existing telemetry/code.
     * Magnetic-switch behavior is intentionally not part of this version.
     */
    public boolean isHomeLimitPressed() {
        return false;
    }

    /** Manually raise the lift. */
    public void raise() {
        liftMotor.setPower(RobotConstants.Lift.MANUAL_POWER);
    }

    /** Manually lower the lift, respecting the software home limit. */
    public void lower() {
        if (getCurrentPosition() <= RobotConstants.Lift.HOME_TICKS) {
            liftMotor.setPower(0);
            return;
        }

        liftMotor.setPower(-RobotConstants.Lift.MANUAL_POWER);
    }

    public void stop() {
        liftMotor.setPower(0);
    }
}
