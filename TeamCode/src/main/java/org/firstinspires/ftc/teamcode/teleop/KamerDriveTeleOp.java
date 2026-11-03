package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;


@Config
@TeleOp(name = "Kamer Drive TeleOp", group = "TeleOp")
public final class KamerDriveTeleOp extends LinearOpMode {

    /*
     * =========================================================
     * GAMEPAD 1
     * =========================================================
     *
     * Left stick:
     *     translation
     *
     * Right stick X:
     *     yaw
     *
     * A:
     *     toggle FAST / SLOW
     *
     * Y:
     *     toggle ROBOT-CENTRIC / FIELD-CENTRIC
     *
     * OPTIONS / START:
     *     reset heading to 0
     *
     *
     * =========================================================
     * STEP 1 OUTPUTS
     * =========================================================
     *
     * targetVxRobot
     *     requested robot-forward velocity, inches/sec
     *
     * targetVyRobot
     *     requested robot-left velocity, inches/sec
     *
     * targetYawRate
     *     requested yaw velocity, radians/sec
     *
     * angleA
     *     angle of requested translation relative to robot heading
     *
     *     0 = robot-forward
     *     +pi/2 = robot-left
     *
     *     If requested translation is zero:
     *         angleA = 0
     */


    // =========================================================
    // TOP SPEEDS
    // =========================================================

    public static double FAST_TRANSLATION_SPEED = 80.0;
    public static double SLOW_TRANSLATION_SPEED = 32.0;

    public static double FAST_YAW_SPEED = Math.PI;
    public static double SLOW_YAW_SPEED = 0.40 * Math.PI;


    // =========================================================
    // RAMP SPEEDS
    //
    // These are ABSOLUTE ramp rates.
    //
    // Slow and fast mode use exactly the same ramp.
    // Therefore slow mode reaches its lower maximum speed sooner.
    // =========================================================

    public static double TRANSLATION_RAMP = 320.0;   // in/s^2
    public static double YAW_RAMP = 4.0 * Math.PI;   // rad/s^2

