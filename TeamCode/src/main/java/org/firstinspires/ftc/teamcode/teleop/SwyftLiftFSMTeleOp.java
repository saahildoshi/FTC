package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.LiftConstants;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

/**
 * SWYFT LIFT FINITE-STATE-MACHINE TELEOP WITH FTC DASHBOARD TUNING.
 *
 * Controls:
 * - A: close the arm/claw and return the lift to HOME
 * - Y: move the lift to HIGH; the arm/claw opens after the lift reaches target
 *
 * IMPORTANT:
 * The current LiftSubsystem implementation makes moveHome() use the REV magnetic
 * limit switch. Keeping the TeleOp on the established subsystem methods also
 * prevents method-name mismatches between older and newer copies of the project.
 */
@TeleOp(name = "Swyft Lift FSM Dashboard", group = "Testing")
public final class SwyftLiftFSMTeleOp extends LinearOpMode {

    private enum LiftState {
        NOT_HOMED,
        MOVING_TO_HOME,
        HOME,
        MOVING_TO_HIGH,
        HIGH,
        HOMING_ERROR
    }

    @Override
    public void runOpMode() throws InterruptedException {

        RobotHardware robot = new RobotHardware();
        robot.initVerifiedHardware(hardwareMap);

        LiftSubsystem lift = new LiftSubsystem(robot);
        ClawSubsystem claw = new ClawSubsystem(robot);

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry());

        ElapsedTime homingTimer = new ElapsedTime();

        LiftState state = lift.isHomed()
                ? LiftState.HOME
                : LiftState.NOT_HOMED;

        // Safe startup condition.
        claw.close();

        telemetry.addLine("Swyft Lift FSM ready");
        telemetry.addLine("A = Home | Y = High");
        telemetry.addData("Home switch", lift.isHomeLimitPressed());
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            lift.stop();
            return;
        }

        boolean previousA = false;
        boolean previousY = false;

        while (opModeIsActive()) {

            boolean aPressed = gamepad1.a && !previousA;
            boolean yPressed = gamepad1.y && !previousY;

            // ==========================================================
            // DRIVER COMMANDS
            // ==========================================================

            if (aPressed) {
                // Close the arm before lowering the lift.
                claw.close();

                // In the updated LiftSubsystem, moveHome() starts the
                // non-blocking REV magnetic-switch homing routine.
                lift.moveHome();

                homingTimer.reset();
                state = LiftState.MOVING_TO_HOME;
            }

            if (yPressed && lift.isHomed()) {
                claw.close();
                lift.moveHigh();
                state = LiftState.MOVING_TO_HIGH;
            }

            // ==========================================================
            // FINITE STATE MACHINE
            // ==========================================================

            switch (state) {

                case NOT_HOMED:
                    claw.close();
                    break;

                case MOVING_TO_HOME:
                    claw.close();

                    // The REV switch is the preferred indication of physical home.
                    // getCurrentPosition()==HOME_TICKS is retained as a compatibility
                    // fallback for an older local copy of LiftSubsystem.
                    boolean reachedHome =
                            lift.isHomed()
                                    && (lift.isHomeLimitPressed()
                                    || lift.getCurrentPosition() == LiftConstants.HOME_TICKS);

                    if (reachedHome) {
                        state = LiftState.HOME;
                    } else if (homingTimer.seconds()
                            >= LiftConstants.HOMING_TIMEOUT_SECONDS) {
                        lift.stop();
                        state = LiftState.HOMING_ERROR;
                    }
                    break;

                case HOME:
                    // HOME always means arm/claw closed.
                    claw.close();
                    break;

                case MOVING_TO_HIGH:
                    // Keep the arm safely closed while the lift is traveling.
                    claw.close();

                    if (lift.atTarget()) {
                        state = LiftState.HIGH;
                    }
                    break;

                case HIGH:
                    // Open only after the lift reaches HIGH within tolerance.
                    claw.open();
                    break;

                case HOMING_ERROR:
                    claw.close();
                    lift.stop();
                    break;
            }

            // Custom PID / homing logic must run every loop.
            lift.update();

            // ==========================================================
            // DRIVER STATION + FTC DASHBOARD TELEMETRY
            // ==========================================================

            telemetry.addData("FSM State", state);
            telemetry.addData("Lift Target", lift.getTargetPosition());
            telemetry.addData("Lift Position", lift.getCurrentPosition());
            telemetry.addData("Lift Error", lift.getError());
            telemetry.addData("At Target", lift.atTarget());
            telemetry.addData("Homed", lift.isHomed());
            telemetry.addData("Home Switch", lift.isHomeLimitPressed());
            telemetry.addData("Arm/Claw Position", claw.getPosition());

            // These are live-editable in FTC Dashboard.
            telemetry.addData("kP", LiftConstants.kP);
            telemetry.addData("kI", LiftConstants.kI);
            telemetry.addData("kD", LiftConstants.kD);
            telemetry.addData("kG", LiftConstants.kG);
            telemetry.addData("HIGH_TICKS", LiftConstants.HIGH_TICKS);
            telemetry.addData("TOLERANCE_TICKS", LiftConstants.TOLERANCE_TICKS);
            telemetry.addData("HOMING_POWER", LiftConstants.HOMING_POWER);
            telemetry.addData("CLAW_CLOSED", LiftConstants.CLAW_CLOSED_POSITION);
            telemetry.addData("CLAW_OPEN", LiftConstants.CLAW_OPEN_POSITION);
            telemetry.update();

            previousA = gamepad1.a;
            previousY = gamepad1.y;
        }

        lift.stop();
        claw.close();
    }
}
