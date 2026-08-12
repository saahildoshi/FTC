package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

/**
 * DRIVE SUBSYSTEM
 *
 * A subsystem represents one major physical function of the robot. This class
 * wraps the team's existing, tuned Road Runner MecanumDrive instead of replacing
 * it. That means all of the proven wheel/localizer/feedforward tuning remains in
 * drive/MecanumDrive.java.
 *
 * PHYSICAL ROBOT CONNECTION:
 * - MecanumDrive controls the four mecanum drive motors.
 * - Its localizer estimates where the robot is on the FTC field.
 * - Road Runner Actions can use the same drive object for autonomous motion.
 *
 * SOFTWARE CONNECTION:
 * TeleOp or autonomous code can call simple methods here instead of knowing the
 * low-level PoseVelocity2d details every time.
 */
public final class DriveSubsystem {
    private final MecanumDrive drive;

    /**
     * initialPose tells Road Runner where we believe the robot starts on the
     * field. Autonomous should use the real starting tile/heading.
     */
    public DriveSubsystem(HardwareMap hardwareMap, Pose2d initialPose) {
        drive = new MecanumDrive(hardwareMap, initialPose);
    }

    /** Gives autonomous code access to Road Runner's Action builder. */
    public MecanumDrive roadRunner() {
        return drive;
    }

    /**
     * Updates sensors/localization and returns the robot's estimated field pose.
     * Pose = X position, Y position, and heading.
     */
    public Pose2d updateAndGetPose() {
        drive.updatePoseEstimate();
        return drive.localizer.getPose();
    }

    /** Re-zero the heading when the driver wants the current direction to be 0. */
    public void resetHeading() {
        drive.resetHeading();
    }

    /**
     * Robot-centric drive: forward always means toward the robot's front.
     * Useful for basic testing and for drivers who prefer robot-relative control.
     */
    public void robotCentric(double forward, double strafe, double turn) {
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(forward, strafe),
                turn
        ));
    }

    /**
     * Field-centric drive: joystick direction stays aligned to the field even as
     * the robot rotates. The current Road Runner heading rotates the joystick
     * vector from field coordinates into robot coordinates.
     */
    public void fieldCentric(double fieldForward, double fieldStrafe, double turn) {
        drive.updatePoseEstimate();
        Pose2d pose = drive.localizer.getPose();

        Vector2d fieldCommand = new Vector2d(fieldForward, fieldStrafe);
        Vector2d robotCommand = pose.heading.inverse().times(fieldCommand);

        drive.setDrivePowers(new PoseVelocity2d(robotCommand, turn));
    }

    /** Stop commanded chassis movement. */
    public void stop() {
        robotCentric(0.0, 0.0, 0.0);
    }
}
