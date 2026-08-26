package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.LiftConstants;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

/**
 * SWYFT LIFT FINITE-STATE-MACHINE TELEOP WITH FTC DASHBOARD TUNING.
 *
 * Controls:
 * - A: close arm/claw and home the lift using the REV magnetic switch
 * - Y: move lift to HIGH; arm/claw opens automatically after lift reaches target
 *
 * The lift remains non-blocking. lift.update() runs every loop, so drivetrain or
 * other mechanisms can be added later without sleep() or RUN_TO_POSITION blocking.
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

        LiftState state = lift.isHomed()
                ? LiftState.HOME
                : LiftState.NOT_HOMED;

        // Safe startup condition: arm/claw closed until the lift is high.
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

            // ----------------------------------------------------------
            // DRIVER COMMANDS
            // ----------------------------------------------------------
            if (aPressed) {
                // Close first so the mechanism is safe before lowering.
                claw.close();
                lift.home();
                state = LiftState.MOVING_TO_HOME;
            }

            if (yPressed && lift.isHomed()) {
                claw.close();
                lift.moveHigh();
                state = LiftState.MOVING_TO_HIGH;
            }

            // ----------------------------------------------------------
            // FINITE STATE MACHINE
            // ----------------------------------------------------------
            switch (state) {
                case NOT_HOMED:
                    claw.close();
                    break;

                case MOVING_TO_HOME:
                    claw.close();

                    if (lift.didHomingTimeOut()) {
                        state = LiftState.HOMING_ERROR;
                    } else if (lift.isHomed()
                            && !lift.isHoming()
                            && lift.isHomeLimitPressed()) {
                        state = LiftState.HOME;
                    }
                    break;

                case HOME:
                    claw.close();
                    break;

                case MOVING_TO_HIGH:
                    claw.close();

                    if (lift.atTarget()) {
                        state = LiftState.HIGH;
                    }
                    break;

                case HIGH:
                    // Only open after the lift has reached the Dashboard-tunable
                    // HIGH_TICKS target within TOLERANCE_TICKS.
                    claw.open();
                    break;

                case HOMING_ERROR:
                    claw.close();
                    lift.stop();
                    break;
            }

            // PID + magnetic homing logic must run every loop.
            lift.update();

            // ----------------------------------------------------------
            // DRIVER STATION + FTC DASHBOARD TELEMETRY
            // ----------------------------------------------------------
            telemetry.addData("FSM State", state);
            telemetry.addData("Lift Target", lift.getTargetPosition());
            telemetry.addData("Lift Position", lift.getCurrentPosition());
            telemetry.addData("Lift Error", lift.getError());
            telemetry.addData("At Target", lift.atTarget());
            telemetry.addData("Homed", lift.isHomed());
            telemetry.addData("Homing", lift.isHoming());
            telemetry.addData("Home Switch", lift.isHomeLimitPressed());
            telemetry.addData("Homing Timeout", lift.didHomingTimeOut());
            telemetry.addData("PID Output", lift.getPidOutput());
            telemetry.addData("Gravity Output", lift.getGravityOutput());
            telemetry.addData("Motor Power", lift.getMotorPower());
            telemetry.addData("Arm/Claw Position", claw.getPosition());

            telemetry.addData("kP", LiftConstants.kP);
            telemetry.addData("kI", LiftConstants.kI);
            telemetry.addData("kD", LiftConstants.kD);
            telemetry.addData("kG", LiftConstants.kG);
            telemetry.addData("HIGH_TICKS", LiftConstants.HIGH_TICKS);
            telemetry.addData("TOLERANCE_TICKS", LiftConstants.TOLERANCE_TICKS);
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
