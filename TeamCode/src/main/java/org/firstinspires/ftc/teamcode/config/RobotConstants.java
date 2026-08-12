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
 * - OptionalMechanisms contains proposed names only; those devices do not yet
 *   exist in the verified repository and are therefore NOT active.
 */
public final class RobotConstants {
    private RobotConstants() { }

    /** Verified drivetrain hardware from the existing robot code. */
    public static final class HardwareNames {
        public static final String LEFT_FRONT = "leftFront";
        public static final String LEFT_BACK = "leftBack";
        public static final String RIGHT_BACK = "rightBack";
        public static final String RIGHT_FRONT = "rightFront";
        public static final String IMU = "imu";

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
     * Proposed mechanism names only.
     *
     * They are intentionally isolated from active HardwareNames so a student
     * cannot accidentally assume the robot already has these devices.
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
