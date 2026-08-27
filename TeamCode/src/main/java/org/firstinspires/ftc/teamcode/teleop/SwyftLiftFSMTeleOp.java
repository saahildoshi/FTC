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

        LiftState state = lift.isAtHome()
                ? LiftState.HOME
                : LiftState.NOT_HOMED;

        claw.close();

        telemetry.addLine("Swyft Lift FSM ready");
        telemetry.addLine("A = Home | Y = High");
        telemetry.addLine("Either gamepad can control the lift");
        telemetry.addData("Home switch", lift.isAtHome());
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            lift.stop();
            return;
        }

        boolean previousA = false;
        boolean previousY = false;

        while (opModeIsActive()) {

            boolean aNow = gamepad1.a || gamepad2.a;
            boolean yNow = gamepad1.y || gamepad2.y;

            boolean aPressed = aNow && !previousA;
            boolean yPressed = yNow && !previousY;

            // ----------------------------------------------------------
            // DRIVER COMMANDS
            // ----------------------------------------------------------
            if (aPressed) {
                claw.close();
                lift.moveHome();
                homingTimer.reset();
                state = LiftState.MOVING_TO_HOME;
            }

            // HIGH is allowed only after the encoder has been calibrated by
            // actually reaching the magnetic home switch at least once.
            if (yPressed && lift.isHomed()) {
                claw.close();
                lift.moveHigh();
                state = LiftState.MOVING_TO_HIGH;
            }

            // ----------------------------------------------------------
            // UPDATE THE LIFT FIRST.
            // This is important: the magnetic switch must be read and downward
            // motor power stopped BEFORE the FSM evaluates HOME/timeout.
            // ----------------------------------------------------------
            lift.update();

            // ----------------------------------------------------------
            // FINITE STATE MACHINE
            // ----------------------------------------------------------
            switch (state) {
                case NOT_HOMED:
                    claw.close();

                    // If the sensor is physically pressed, HOME is established
                    // immediately by LiftSubsystem.update().
                    if (lift.isAtHome()) {
                        state = LiftState.HOME;
                    }
                    break;

                case MOVING_TO_HOME:
                    claw.close();

                    // THE MAGNETIC SWITCH IS THE HOME CONDITION.
                    // Encoder ticks are intentionally NOT checked here.
                    if (lift.isAtHome()) {
                        lift.stop();
                        state = LiftState.HOME;
                    } else if (lift.didHomingTimeOut()
                            || homingTimer.seconds() >= LiftConstants.HOMING_TIMEOUT_SECONDS) {
                        lift.stop();
                        state = LiftState.HOMING_ERROR;
                    }
                    break;

                case HOME:
                    claw.close();

                    // HOME state represents the physical switch location.
                    // If the lift leaves the switch because another command is
                    // issued, the state will be changed by that command.
                    break;

                case MOVING_TO_HIGH:
                    claw.close();

                    if (lift.atTarget()) {
                        state = LiftState.HIGH;
                    }
                    break;

                case HIGH:
                    claw.open();
                    break;

                case HOMING_ERROR:
                    claw.close();
                    lift.stop();
                    break;
            }

            // ----------------------------------------------------------
            // TELEMETRY
            // ----------------------------------------------------------
            telemetry.addData("FSM State", state);
            telemetry.addData("Gamepad A", aNow);
            telemetry.addData("Gamepad Y", yNow);
            telemetry.addData("Lift Target", lift.getTargetPosition());
            telemetry.addData("Lift Position", lift.getCurrentPosition());
            telemetry.addData("Lift Error", lift.getError());
            telemetry.addData("At Target", lift.atTarget());
            telemetry.addData("Encoder Calibrated", lift.isHomed());
            telemetry.addData("PHYSICAL HOME SWITCH", lift.isAtHome());
            telemetry.addData("TouchSensor isPressed", robot.magneticLimitSwitch.isPressed());
            telemetry.addData("TouchSensor value", robot.magneticLimitSwitch.getValue());
            telemetry.addData("Homing", lift.isHoming());
            telemetry.addData("Homing Timed Out", lift.didHomingTimeOut());
            telemetry.addData("Motor Power", lift.getMotorPower());
            telemetry.addData("Arm/Claw Position", claw.getPosition());

            telemetry.addData("kP", LiftConstants.kP);
            telemetry.addData("kI", LiftConstants.kI);
            telemetry.addData("kD", LiftConstants.kD);
            telemetry.addData("kG", LiftConstants.kG);
            telemetry.addData("HIGH_TICKS", LiftConstants.HIGH_TICKS);
            telemetry.addData("HOMING_POWER", LiftConstants.HOMING_POWER);

            if (yPressed && !lift.isHomed()) {
                telemetry.addLine("Y IGNORED: Home the lift with A first");
            }

            if (lift.isAtHome()) {
                telemetry.addLine("HOME SENSOR PRESSED - DOWNWARD POWER BLOCKED");
            }

            telemetry.update();

            previousA = aNow;
            previousY = yNow;
        }

        lift.stop();
        claw.close();
    }
}
