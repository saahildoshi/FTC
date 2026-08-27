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
 * IMPORTANT HOME RULE:
 * The REV magnetic limit switch is the ONLY authority for the physical HOME
 * position. Encoder ticks do not determine whether the lift is physically home.
 * When the switch is pressed, the encoder is reset so all other lift positions
 * can still be measured from that known physical location.
 */
public final class LiftSubsystem {

    private final DcMotorEx liftMotor;
    private final TouchSensor homeSwitch;
    private final CustomPIDController pid;
    private final ElapsedTime homingTimer = new ElapsedTime();

    private int targetPosition = LiftConstants.HOME_TICKS;

    // "homed" means the encoder has been calibrated from the physical switch.
    // It does NOT mean the lift is currently sitting at home.
    private boolean homed = false;
    private boolean homing = false;
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
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pid = new CustomPIDController(
                LiftConstants.kP,
                LiftConstants.kI,
                LiftConstants.kD);
        pid.setTarget(LiftConstants.HOME_TICKS);

        // If the lift starts while physically on the switch, that switch establishes
        // home immediately and the encoder is calibrated from that physical position.
        if (isAtHome()) {
            resetEncoderAtHome();
            previousHomeState = true;
        }
    }

    /** Must be called every active OpMode loop during automatic/PID control. */
    public void update() {
        boolean homePressed = isAtHome();

        pid.setPID(LiftConstants.kP, LiftConstants.kI, LiftConstants.kD);

        if (homing) {
            runHoming(homePressed);
            previousHomeState = homePressed;
            return;
        }

        // The physical switch is authoritative. Whenever the lift newly reaches
        // the magnet, establish that exact location as encoder zero.
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

        // PHYSICAL HOME SAFETY:
        // Do not allow negative/downward power while the magnetic switch is pressed.
        // There is intentionally NO encoder-tick lower-limit check here.
        if (homePressed && motorPower < 0.0) {
            motorPower = 0.0;
        }

        // Encoder ticks are still useful for the upper software limit after the lift
        // has been calibrated by the physical home switch.
        if (homed
                && currentPosition >= LiftConstants.MAX_TICKS
                && motorPower > 0.0) {
            motorPower = 0.0;
        }

        liftMotor.setPower(motorPower);
        previousHomeState = homePressed;
    }

    /** Begin a non-blocking search for the physical REV magnetic home switch. */
    public void home() {
        homingTimedOut = false;
        pid.reset();

        // If the switch is already pressed, we are physically home right now.
        if (isAtHome()) {
            resetEncoderAtHome();
            homing = false;
            return;
        }

        homing = true;
        homingTimer.reset();
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

    /**
     * Called only when the magnetic switch establishes the physical HOME location.
     * HOME_TICKS is simply the encoder coordinate assigned to that physical point.
     */
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

    /** Return to HOME by searching for the physical magnetic switch. */
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
        return getCurrentPosition();
    }

    public double getHeightFraction() {
        if (LiftConstants.MAX_TICKS <= LiftConstants.HOME_TICKS) {
            return 0.0;
        }

        return Math.max(
                0.0,
                Math.min(
                        1.0,
                        (double) (getCurrentPosition() - LiftConstants.HOME_TICKS)
                                / (LiftConstants.MAX_TICKS - LiftConstants.HOME_TICKS)));
    }

    public double getError() {
        return targetPosition - liftMotor.getCurrentPosition();
    }

    public boolean atTarget() {
        return !homing
                && Math.abs(getError()) < LiftConstants.TOLERANCE_TICKS;
    }

    /**
     * Manual encoder reset retained for compatibility. Physical homing should normally
     * use home()/moveHome() so the magnetic switch establishes the zero correctly.
     */
    public void resetEncoder() {
        liftMotor.setPower(0.0);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        targetPosition = LiftConstants.HOME_TICKS;
        pid.reset();
        pid.setTarget(LiftConstants.HOME_TICKS);
    }

    /** True after the encoder has been calibrated from the magnetic switch. */
    public boolean isHomed() {
        return homed;
    }

    /** True only while the lift is currently physically at the HOME switch. */
    public boolean isAtHome() {
        return homeSwitch != null && homeSwitch.isPressed();
    }

    public boolean isHoming() {
        return homing;
    }

    public boolean didHomingTimeOut() {
        return homingTimedOut;
    }

    /** Compatibility name used by existing telemetry and Actions. */
    public boolean isHomeLimitPressed() {
        return isAtHome();
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

    /** Manually raise the lift. */
    public void raise() {
        homing = false;
        motorPower = Math.abs(LiftConstants.MANUAL_POWER);

        if (homed && getCurrentPosition() >= LiftConstants.MAX_TICKS) {
            motorPower = 0.0;
        }

        liftMotor.setPower(motorPower);
    }

    /**
     * Manually lower at full negative power. The ONLY lower stop is the physical
     * magnetic HOME switch; encoder ticks do not determine physical home.
     */
    public void lower() {
        homing = false;

        if (isAtHome()) {
            motorPower = 0.0;
            liftMotor.setPower(0.0);
            return;
        }

        motorPower = -1.0;
        liftMotor.setPower(-1.0);
    }

    public void stop() {
        homing = false;
        motorPower = 0.0;
        liftMotor.setPower(0.0);
    }
}
