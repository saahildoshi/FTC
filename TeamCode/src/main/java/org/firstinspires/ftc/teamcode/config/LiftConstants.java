package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;

/**
 * LIVE FTC DASHBOARD TUNING VALUES FOR THE SWYFT LIFT.
 *
 * Every public static field in this @Config class can be changed from FTC
 * Dashboard while an OpMode is running. Once the team finds reliable values,
 * copy those tuned values back here so they remain the defaults after restart.
 */
@Config
public final class LiftConstants {
    private LiftConstants() { }

    // ----------------------------------------------------------------------
    // CUSTOM PID
    // ----------------------------------------------------------------------
    public static double kP = 0.003;
    public static double kI = 0.0;
    public static double kD = 0.0001;

    // Constant upward power used to counter gravity while the lift is raised.
    public static double kG = 0.08;

    // ----------------------------------------------------------------------
    // MOTOR OUTPUT LIMITS
    // ----------------------------------------------------------------------
    public static double MAX_UP_POWER = 1.0;
    public static double MAX_DOWN_POWER = 0.70;
    public static double MANUAL_POWER = 0.75;

    // ----------------------------------------------------------------------
    // REV MAGNETIC HOME SWITCH
    // ----------------------------------------------------------------------
    // Negative power is assumed to move the lift toward the bottom/home switch.
    public static double HOMING_POWER = -0.20;

    // Stops a failed homing attempt instead of driving downward forever.
    public static double HOMING_TIMEOUT_SECONDS = 5.0;

    // ----------------------------------------------------------------------
    // SWYFT LIFT POSITIONS - ENCODER TICKS
    // ----------------------------------------------------------------------
    public static int HOME_TICKS = 0;
    public static int LOW_TICKS = 500;
    public static int MID_TICKS = 1100;
    public static int HIGH_TICKS = 1800;
    public static int MAX_TICKS = 2100;

    public static int TOLERANCE_TICKS = 25;

    // Below this height, gravity compensation is disabled so the motor does not
    // push against the bottom of the mechanism.
    public static int GRAVITY_ENABLE_TICKS = 50;

    // ----------------------------------------------------------------------
    // ARM / CLAW SERVO POSITIONS
    // ----------------------------------------------------------------------
    // The team's active servo is configured as "claw". These values control the
    // open/closed arm behavior used by the lift finite-state machine.
    public static double CLAW_CLOSED_POSITION = 0.0;
    public static double CLAW_OPEN_POSITION = 0.60;
}
