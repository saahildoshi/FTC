package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;

/**
 * LIVE FTC DASHBOARD TUNING VALUES FOR THE SHOOTER FLYWHEEL.
 *
 * The shooter uses encoder velocity feedback rather than position feedback.
 * Feedforward supplies the baseline power needed to hold the requested speed,
 * while PID corrects velocity error caused by friction, battery changes, and
 * the sudden load of launching a ball.
 */
@Config
public final class ShooterConstants {
    private ShooterConstants() { }

    // ----------------------------------------------------------------------
    // VELOCITY PID
    // Error units are encoder ticks/second.
    // Tune these live in FTC Dashboard for velocity error in ticks/second.
    // ----------------------------------------------------------------------
    public static double kP = 0.003;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double PID_INTEGRAL_LIMIT = 500.0;

    // Velocity feedforward: normalized motor power per target tick/second.
    public static double kF = 0.0003;

    // ----------------------------------------------------------------------
    // TARGET VELOCITY
    // ----------------------------------------------------------------------
    public static final double ABSOLUTE_MAX_TARGET_VELOCITY_TICKS_PER_SECOND =
            1335;
    public static double TARGET_VELOCITY_TICKS_PER_SECOND = 1235;
    public static double MAX_TARGET_VELOCITY_TICKS_PER_SECOND = 1335;

    // Shooter is considered ready when actual speed is this close to target.
    public static double VELOCITY_TOLERANCE_TICKS_PER_SECOND = 50.0;

    // Automatic gate hysteresis. The gate opens only after the flywheel reaches
    // the upper threshold and closes only after it falls below the lower one.
    public static double GATE_OPEN_VELOCITY_TICKS_PER_SECOND = 1100.0;
    public static double GATE_CLOSE_VELOCITY_TICKS_PER_SECOND = 900.0;

    // ----------------------------------------------------------------------
    // OUTPUT / VOLTAGE COMPENSATION
    // ----------------------------------------------------------------------
    public static double MAX_POWER = 1.0;
    public static double NOMINAL_VOLTAGE = 12.0;
}
