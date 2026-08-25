package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

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

    /**
     * Reuse the motor already mapped by RobotHardware instead of looking it up
     * again. This keeps hardware ownership consistent across the project.
     */
    public LiftSubsystem(RobotHardware robot) {
        liftMotor = robot.leftLift;

        // BRAKE resists free movement when motor power is zero. This is commonly
        // useful for lifts, although the real mechanism still determines whether
        // additional holding power/gravity compensation is needed.
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Preserve encoder position and use it for closed-loop preset movement.
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Begin moving toward an encoder target without blocking the OpMode thread.
     * The target is clamped to the software limits defined in RobotConstants.
     */
    public void moveTo(int requestedTicks) {
        int safeTarget = Math.max(
                RobotConstants.Lift.HOME_TICKS,
                Math.min(RobotConstants.Lift.MAX_TICKS, requestedTicks)
        );

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

    /** Target currently stored in the motor controller. */
    public int getTargetPosition() {
        return liftMotor.getTargetPosition();
    }

    /**
     * Returns true when the lift is close enough to its requested position that
     * an autonomous Action can move on to the next step.
     */
    public boolean atTarget() {
        return Math.abs(getTargetPosition() - getCurrentPosition())
                <= RobotConstants.Lift.TOLERANCE_TICKS;
    }

    /** Immediately remove motor power. */
    public void stop() {
        liftMotor.setPower(0.0);
    }
    public void raise(){liftMotor.setPower(1.0);}
    public void lower(){liftMotor.setPower(-1.0);}
}
