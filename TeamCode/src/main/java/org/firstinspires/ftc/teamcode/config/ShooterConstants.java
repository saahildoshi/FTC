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
    // Characterized from 2100 ticks/s at raw power 1.0 and 12.11 V.
    // kP = 0.0005 adds 0.05 power for a 100 ticks/s velocity error.
    // ----------------------------------------------------------------------
    public static double kP = 0.00050;
    public static double kI = 0.0;
    public static double kD = 0.0;

    // Velocity feedforward: normalized motor power per target tick/second.
    // kF = V_loaded / (V_nominal * measured velocity)
    //    = 12.11 / (12.0 * 2100) ~= 0.0004806.
    // At 1200 ticks/s this requests ~0.577 power at the 12 V reference,
    // leaving output available for feedback recovery.
    public static double kF = 0.0004806;

    // ----------------------------------------------------------------------
    // TARGET VELOCITY
    // ----------------------------------------------------------------------
    public static final double ABSOLUTE_MAX_TARGET_VELOCITY_TICKS_PER_SECOND =
            1200.0;
    public static double TARGET_VELOCITY_TICKS_PER_SECOND = 1200.0;
    public static double MAX_TARGET_VELOCITY_TICKS_PER_SECOND = 1200.0;

    // Shooter is considered ready when actual speed is this close to target.
    public static double VELOCITY_TOLERANCE_TICKS_PER_SECOND = 50.0;

    // ----------------------------------------------------------------------
    // OUTPUT / VOLTAGE COMPENSATION
    // ----------------------------------------------------------------------
    public static double MAX_POWER = 1.0;
    public static double NOMINAL_VOLTAGE = 12.0;
}
