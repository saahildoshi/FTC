package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.config.ShooterConstants;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.util.CustomPIDController;

/**
 * CLOSED-LOOP SHOOTER VELOCITY CONTROLLER.
 *
 * Similar to LiftSubsystem, this class uses the team's CustomPIDController and
 * Dashboard-tunable constants. The important difference is that the feedback
 * variable is motor velocity (ticks/second), not encoder position.
 *
 * Call update() every OpMode loop while the shooter is enabled.
 */
public final class ShooterSubsystem {

    private final DcMotorEx shooterMotor;
    private final CustomPIDController pid;
    private final RobotHardware robot;

    private double targetVelocity = 0.0;
    private boolean enabled = false;

    private double currentVelocity = 0.0;
    private double velocityError = 0.0;
    private double pidOutput = 0.0;
    private double feedforwardOutput = 0.0;
    private double voltageCompensation = 1.0;
    private double motorPower = 0.0;

    public ShooterSubsystem(RobotHardware robot) {
        this.robot = robot;
        shooterMotor = robot.shooter;

        if (shooterMotor == null) {
            throw new IllegalStateException(
                    "Shooter motor is not initialized. Call RobotHardware.initShooterHardware() first.");
        }

        shooterMotor.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pid = new CustomPIDController(
                ShooterConstants.kP,
                ShooterConstants.kI,
                ShooterConstants.kD);

        pid.setTarget(0.0);
    }

    /**
     * Must be called every active OpMode loop while using closed-loop shooter control.
     */
    public void update() {
        currentVelocity = Math.abs(shooterMotor.getVelocity());

        if (!enabled || targetVelocity <= 0.0) {
            pidOutput = 0.0;
            feedforwardOutput = 0.0;
            velocityError = 0.0;
            voltageCompensation = 1.0;
            motorPower = 0.0;
            shooterMotor.setPower(0.0);
            return;
        }

        // Allow live FTC Dashboard PID tuning just like the lift.
        pid.setPID(
                ShooterConstants.kP,
                ShooterConstants.kI,
                ShooterConstants.kD);

        pid.setTarget(targetVelocity);

        velocityError = targetVelocity - currentVelocity;
        pidOutput = pid.calculate(currentVelocity);

        // Baseline power needed to maintain the requested flywheel speed.
        feedforwardOutput = ShooterConstants.kF * targetVelocity;

        // Keep the effective motor voltage more consistent as battery voltage changes.
        double batteryVoltage = getBatteryVoltage();
        if (batteryVoltage > 1.0) {
            voltageCompensation =
                    ShooterConstants.NOMINAL_VOLTAGE / batteryVoltage;
        } else {
            voltageCompensation = 1.0;
        }

        // When a ball loads the flywheel, velocity drops, error becomes positive,
        // and the PID term immediately adds recovery power above the feedforward.
        motorPower =
                (feedforwardOutput + pidOutput) * voltageCompensation;

        motorPower = Range.clip(
                motorPower,
                0.0,
                Math.abs(ShooterConstants.MAX_POWER));

        shooterMotor.setPower(motorPower);
    }

    /** Start the shooter at the Dashboard-configured target velocity. */
    public void start() {
        setTargetVelocity(ShooterConstants.TARGET_VELOCITY_TICKS_PER_SECOND);
    }

    /** Set a new closed-loop target velocity in encoder ticks/second. */
    public void setTargetVelocity(double ticksPerSecond) {
        double safeTarget = Range.clip(
                Math.abs(ticksPerSecond),
                0.0,
                Math.abs(ShooterConstants.MAX_TARGET_VELOCITY_TICKS_PER_SECOND));

        if (!enabled || Math.abs(safeTarget - targetVelocity) > 0.001) {
            pid.reset();
        }

        targetVelocity = safeTarget;
        pid.setTarget(targetVelocity);
        enabled = targetVelocity > 0.0;
    }

    public void stop() {
        enabled = false;
        targetVelocity = 0.0;
        pid.reset();
        pid.setTarget(0.0);

        currentVelocity = Math.abs(shooterMotor.getVelocity());
        velocityError = 0.0;
        pidOutput = 0.0;
        feedforwardOutput = 0.0;
        motorPower = 0.0;

        shooterMotor.setPower(0.0);
    }

    public boolean isEnabled() {
        return enabled;
    }

    /**
     * True when the shooter is close enough to target speed to launch consistently.
     */
    public boolean isReadyToShoot() {
        return enabled
                && Math.abs(getVelocityError())
                <= ShooterConstants.VELOCITY_TOLERANCE_TICKS_PER_SECOND;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getCurrentVelocity() {
        return Math.abs(shooterMotor.getVelocity());
    }

    public double getVelocityError() {
        return targetVelocity - getCurrentVelocity();
    }

    public double getPidOutput() {
        return pidOutput;
    }

    public double getFeedforwardOutput() {
        return feedforwardOutput;
    }

    public double getVoltageCompensation() {
        return voltageCompensation;
    }

    public double getMotorPower() {
        return motorPower;
    }

    public double getBatteryVoltage() {
        if (robot.voltageSensor == null) {
            return ShooterConstants.NOMINAL_VOLTAGE;
        }

        return robot.voltageSensor.getVoltage();
    }
}
