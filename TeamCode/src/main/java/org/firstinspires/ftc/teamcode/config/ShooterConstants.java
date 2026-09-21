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
    // These are safe STARTING values and should be tuned on the physical robot.
    // ----------------------------------------------------------------------
    public static double kP = 0.00035;
    public static double kI = 0.00005;
    public static double kD = 0.000002;

    // Velocity feedforward: normalized motor power per target tick/second.
    // Example: 1800 ticks/s * 0.00035 ~= 0.63 baseline power at 12 V.
    public static double kF = 0.00035;

    // ----------------------------------------------------------------------
    // TARGET VELOCITY
    // ----------------------------------------------------------------------
    public static double TARGET_VELOCITY_TICKS_PER_SECOND = 1800.0;
    public static double MAX_TARGET_VELOCITY_TICKS_PER_SECOND = 3000.0;

    // Shooter is considered ready when actual speed is this close to target.
    public static double VELOCITY_TOLERANCE_TICKS_PER_SECOND = 50.0;

    // ----------------------------------------------------------------------
    // OUTPUT / VOLTAGE COMPENSATION
    // ----------------------------------------------------------------------
    public static double MAX_POWER = 1.0;
    public static double NOMINAL_VOLTAGE = 12.0;
}
