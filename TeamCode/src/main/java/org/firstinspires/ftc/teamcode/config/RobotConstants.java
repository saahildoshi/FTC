package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public final class RobotConstants {
    private RobotConstants() { }

    public static final class HardwareNames {
        public static final String LEFT_FRONT = "leftFront";
        public static final String LEFT_BACK = "leftBack";
        public static final String RIGHT_BACK = "rightBack";
        public static final String RIGHT_FRONT = "rightFront";
        public static final String IMU = "imu";
        public static final String LIFT = "leftLift";
        public static final String MAGNETIC_LIMIT_SWITCH = "magneticLimitSwitch";
        public static final String CLAW = "claw";
        public static final String INTAKE = "intake";
        public static final String SERVO_270 = "servo270";

        private HardwareNames() { }
    }

    public static final class Imu {
        public static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING =
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        public static final RevHubOrientationOnRobot.UsbFacingDirection USB_FACING =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;

        private Imu() { }
    }

    public static final class TeleOp {
        public static final double SLOW_MODE_MULTIPLIER = 0.40;
        public static final double HEADING_KP = 1.50;
        public static final double HEADING_KI = 0.00;
        public static final double HEADING_KD = 0.05;
        public static final double HEADING_HOLD_DELAY_SECONDS = 0.20;
        public static final double DRIVE_ACCEL = 2.50;
        public static final double ROTATION_ACCEL = 3.00;

        private TeleOp() { }
    }

    public static final class Lift {
        public static final int HOME_TICKS = 0;
        public static final int LOW_TICKS = 500;
        public static final int MID_TICKS = 1100;
        public static final int HIGH_TICKS = 1800;
        public static final int MAX_TICKS = 2100;
        public static final double MOVE_POWER = 0.75;
        public static final double MANUAL_POWER = 0.75;
        public static final int TOLERANCE_TICKS = 25;

        private Lift() { }
    }

    public static final class Claw {
        public static final double OPEN_POSITION = 0.60;
        public static final double CLOSED_POSITION = 0.0;

        private Claw() { }
    }

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
