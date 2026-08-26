package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.config.LiftConstants;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.util.CustomPIDController;

/**
 * ACTIVE SINGLE-MOTOR SWYFT LIFT SUBSYSTEM
 *
 * Features:
 * - Custom PID controller
 * - FTC Dashboard live tuning through LiftConstants
 * - Gravity feedforward
 * - REV magnetic switch homing
 * - Automatic encoder zero at physical home
 * - Software upper/lower limits
 * - Non-blocking homing suitable for TeleOp FSMs and Road Runner Actions
 */
public final class LiftSubsystem {

    private final DcMotorEx liftMotor;
    private final TouchSensor homeSwitch;
    private final CustomPIDController pid;
    private final ElapsedTime homingTimer = new ElapsedTime();

    private int targetPosition = LiftConstants.HOME_TICKS;

    private boolean homing = false;
    private boolean homed = false;
    private boolean homingTimedOut = false;
    private boolean previousHomeState = false;

    private double pidOutput = 0.0;
    private double gravityOutput = 0.0;
    private double motorPower = 0.0;

    public LiftSubsystem(RobotHardware robot) {
        liftMotor = robot.leftLift;
        homeSwitch = robot.magneticLimitSwitch;

        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Custom PID controls motor power. The encoder is still readable in this mode.
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pid = new CustomPIDController(
                LiftConstants.kP,
                LiftConstants.kI,
                LiftConstants.kD);
        pid.setTarget(LiftConstants.HOME_TICKS);

        // If the robot powers on while already sitting on the magnet, establish a
        // valid physical zero immediately without moving the mechanism.
        if (isHomeLimitPressed()) {
            resetEncoderAtHome();
            previousHomeState = true;
        }
    }

    /** Must be called every active OpMode loop. */
    public void update() {
        boolean homePressed = isHomeLimitPressed();

        // Read live PID values every loop so Dashboard edits take effect immediately.
        pid.setPID(LiftConstants.kP, LiftConstants.kI, LiftConstants.kD);

        if (homing) {
            runHoming(homePressed);
            previousHomeState = homePressed;
            return;
        }

        // Re-zero when the carriage physically arrives at home. Edge detection keeps
        // the encoder from being reset continuously while sitting over the magnet.
        if (homePressed && !previousHomeState) {
            resetEncoderAtHome();
        }

        int currentPosition = liftMotor.getCurrentPosition();

        pidOutput = pid.calculate(currentPosition);

        gravityOutput = 0.0;
        if (targetPosition > LiftConstants.GRAVITY_ENABLE_TICKS
                || currentPosition > LiftConstants.GRAVITY_ENABLE_TICKS) {
            gravityOutput = LiftConstants.kG;
        }

        motorPower = pidOutput + gravityOutput;
        motorPower = Range.clip(
                motorPower,
                -Math.abs(LiftConstants.MAX_DOWN_POWER),
                Math.abs(LiftConstants.MAX_UP_POWER));

        // Physical home switch always wins over a downward command.
        if (homePressed && motorPower < 0.0) {
            motorPower = 0.0;
        }

        // Software lower limit is trusted only after the lift has physically homed.
        if (homed
                && currentPosition <= LiftConstants.HOME_TICKS
                && motorPower < 0.0) {
            motorPower = 0.0;
        }

        if (currentPosition >= LiftConstants.MAX_TICKS && motorPower > 0.0) {
            motorPower = 0.0;
        }

        liftMotor.setPower(motorPower);
        previousHomeState = homePressed;
    }

    /** Begin a non-blocking search for the REV magnetic home switch. */
    public void home() {
        homing = true;
        homingTimedOut = false;
        homingTimer.reset();
        pid.reset();
    }

    private void runHoming(boolean homePressed) {
        pidOutput = 0.0;
        gravityOutput = 0.0;

        if (homePressed) {
            motorPower = 0.0;
            liftMotor.setPower(0.0);
            resetEncoderAtHome();
            homing = false;
            homingTimedOut = false;
            return;
        }

        if (homingTimer.seconds() >= LiftConstants.HOMING_TIMEOUT_SECONDS) {
            motorPower = 0.0;
            liftMotor.setPower(0.0);
            homing = false;
            homingTimedOut = true;
            return;
        }

        motorPower = Range.clip(LiftConstants.HOMING_POWER, -1.0, 0.0);
        liftMotor.setPower(motorPower);
    }

    private void resetEncoderAtHome() {
        liftMotor.setPower(0.0);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        targetPosition = LiftConstants.HOME_TICKS;
        pid.reset();
        pid.setTarget(LiftConstants.HOME_TICKS);

        homed = true;
        motorPower = 0.0;
    }

    public void moveTo(int target) {
        homing = false;
        homingTimedOut = false;

        targetPosition = Range.clip(
                target,
                LiftConstants.HOME_TICKS,
                LiftConstants.MAX_TICKS);

        pid.setTarget(targetPosition);
    }

    /** Return to physical home using the magnetic switch rather than encoder-only zero. */
    public void moveHome() {
        home();
    }

    public void moveLow() {
        moveTo(LiftConstants.LOW_TICKS);
    }

    public void moveMid() {
        moveTo(LiftConstants.MID_TICKS);
    }

    public void moveHigh() {
        moveTo(LiftConstants.HIGH_TICKS);
    }

    public int getTargetPosition() {
        return targetPosition;
    }

    public int getCurrentPosition() {
        return liftMotor.getCurrentPosition();
    }

    public int getHeightTicks() {
        return Math.max(LiftConstants.HOME_TICKS, getCurrentPosition());
    }

    public double getHeightFraction() {
        if (LiftConstants.MAX_TICKS <= LiftConstants.HOME_TICKS) {
            return 0.0;
        }

        return Math.min(
                1.0,
                (double) (getHeightTicks() - LiftConstants.HOME_TICKS)
                        / (LiftConstants.MAX_TICKS - LiftConstants.HOME_TICKS));
    }

    public double getError() {
        return targetPosition - liftMotor.getCurrentPosition();
    }

    public boolean atTarget() {
        return !homing
                && Math.abs(getError()) < LiftConstants.TOLERANCE_TICKS;
    }

    public void resetEncoder() {
        resetEncoderAtHome();
    }

    public boolean isHomed() {
        return homed;
    }

    public boolean isHoming() {
        return homing;
    }

    public boolean didHomingTimeOut() {
        return homingTimedOut;
    }

    public boolean isHomeLimitPressed() {
        return homeSwitch != null && homeSwitch.isPressed();
    }

    public double getPidOutput() {
        return pidOutput;
    }

    public double getGravityOutput() {
        return gravityOutput;
    }

    public double getMotorPower() {
        return motorPower;
    }

    /** Manually raise the lift. Manual control cancels PID/homing for that moment. */
    public void raise() {
        homing = false;
        motorPower = Math.abs(LiftConstants.MANUAL_POWER);

        if (getCurrentPosition() >= LiftConstants.MAX_TICKS) {
            motorPower = 0.0;
        }

        liftMotor.setPower(motorPower);
    }

    /** Manually lower the lift while respecting the physical home switch. */
    public void lower() {
        homing = false;

        if (isHomeLimitPressed()
                || (homed && getCurrentPosition() <= LiftConstants.HOME_TICKS)) {
            motorPower = 0.0;
            liftMotor.setPower(0.0);
            return;
        }

        motorPower = -Math.abs(LiftConstants.MANUAL_POWER);
        liftMotor.setPower(motorPower);
    }

    public void stop() {
        homing = false;
        motorPower = 0.0;
        liftMotor.setPower(0.0);
    }
}
