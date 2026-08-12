package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

/**
 * ROBOT CONSTANTS / CONFIGURATION MAP
 *
 * This class is the team's single place for names and values that describe the
 * physical robot. Keeping these values here prevents students from scattering
 * configuration strings and tuning values throughout many OpModes.
 *
 * HOW THIS RELATES TO THE ROBOT:
 * - HardwareNames must match the names entered in the FTC Robot Controller app.
 * - IMU orientation describes how the Control Hub is physically mounted.
 * - TeleOp values change how the robot feels to the driver.
 * - Lift and Claw contain the currently active mechanism settings.
 * - OptionalMechanisms still preserves other mechanism choices for later.
 */
public final class RobotConstants {
    private RobotConstants() { }

    /** Verified drivetrain hardware plus the active lift and claw. */
    public static final class HardwareNames {
        public static final String LEFT_FRONT = "leftFront";
        public static final String LEFT_BACK = "leftBack";
        public static final String RIGHT_BACK = "rightBack";
        public static final String RIGHT_FRONT = "rightFront";
        public static final String IMU = "imu";

        // Single active lift motor selected from the optional two-motor design.
        public static final String LIFT = "leftLift";

        // Active positional servo used to open and close the claw.
        public static final String CLAW = "claw";

        private HardwareNames() { }
    }

    /**
     * Physical Control Hub orientation already used by the team's code.
     * Road Runner/field-centric driving needs a correct heading, so these values
     * must match how the hub actually sits on the chassis.
     */
    public static final class Imu {
        public static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING =
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        public static final RevHubOrientationOnRobot.UsbFacingDirection USB_FACING =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;

        private Imu() { }
    }

    /** Driver-control values derived from the existing TeleOp behavior. */
    public static final class TeleOp {
        // Reduces drive output for precision movement.
        public static final double SLOW_MODE_MULTIPLIER = 0.40;

        // Heading-hold PID: corrects unwanted rotation when the driver releases turn.
        public static final double HEADING_KP = 1.50;
        public static final double HEADING_KI = 0.00;
        public static final double HEADING_KD = 0.05;
        public static final double HEADING_HOLD_DELAY_SECONDS = 0.20;

        // Slew/acceleration limits smooth sudden joystick changes.
        public static final double DRIVE_ACCEL = 2.50;
        public static final double ROTATION_ACCEL = 3.00;

        private TeleOp() { }
    }

    /**
     * ACTIVE SINGLE-MOTOR LIFT SETTINGS
     *
     * Encoder positions are still starting values until the team measures the
     * real lift. Positive encoder ticks are assumed to move the lift upward.
     */
    public static final class Lift {
        // Bottom/base position of the lift.
        public static final int HOME_TICKS = 0;

        // Example intermediate heights for future TeleOp presets.
        public static final int LOW_TICKS = 500;
        public static final int MID_TICKS = 1100;

        // Highest position used by the sample autonomous.
        public static final int HIGH_TICKS = 1800;

        // Software ceiling intended to keep the motor from commanding past the mechanism.
        public static final int MAX_TICKS = 2100;

        // Motor output used by RUN_TO_POSITION.
        public static final double MOVE_POWER = 0.75;

        // An autonomous lift Action ends when the encoder is this close to target.
        public static final int TOLERANCE_TICKS = 25;

        private Lift() { }
    }

    /**
     * ACTIVE CLAW SERVO SETTINGS
     *
     * FTC positional servos use the range 0.0 to 1.0. These positions are still
     * starting values and must be checked against the physical linkage.
     */
    public static final class Claw {
        public static final double OPEN_POSITION = 0.70;
        public static final double CLOSED_POSITION = 0.30;

        private Claw() { }
    }

    /**
     * Proposed mechanism names only.
     *
     * This section is intentionally preserved so the team can still choose a
     * second lift motor, arm, intake, or CR-servo intake later.
     */
    public static final class OptionalMechanisms {
        public static final String LEFT_LIFT = "leftLift";
        public static final String RIGHT_LIFT = "rightLift";
        public static final String ARM = "arm";
        public static final String INTAKE = "intake";
        public static final String CLAW = "claw";
        public static final String CR_SERVO_INTAKE = "intakeServo";

        private OptionalMechanisms() { }
    }
}
