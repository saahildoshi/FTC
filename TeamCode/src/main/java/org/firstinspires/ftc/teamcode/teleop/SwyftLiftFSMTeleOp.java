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

        LiftState state = lift.isHomeLimitPressed()
                ? LiftState.HOME
                : LiftState.NOT_HOMED;

        claw.close();

        telemetry.addLine("Swyft Lift FSM ready");
        telemetry.addLine("A = Home | Y = High");
        telemetry.addLine("Either gamepad can control the lift");
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

            boolean aNow = gamepad1.a || gamepad2.a;
            boolean yNow = gamepad1.y || gamepad2.y;

            boolean aPressed = aNow && !previousA;
            boolean yPressed = yNow && !previousY;

            if (aPressed) {
                claw.close();
                lift.moveHome();
                homingTimer.reset();
                state = LiftState.MOVING_TO_HOME;
            }

            if (yPressed && lift.isHomed()) {
                claw.close();
                lift.moveHigh();
                state = LiftState.MOVING_TO_HIGH;
            }

            // Read the magnetic switch and update lift control before evaluating FSM state.
            lift.update();

            switch (state) {
                case NOT_HOMED:
                    claw.close();

                    if (lift.isHomeLimitPressed()) {
                        state = LiftState.HOME;
                    }
                    break;

                case MOVING_TO_HOME:
                    claw.close();

                    // Magnetic limit switch is the physical HOME condition.
                    if (lift.isHomeLimitPressed()) {
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

            telemetry.addData("FSM State", state);
            telemetry.addData("Gamepad A", aNow);
            telemetry.addData("Gamepad Y", yNow);
            telemetry.addData("Lift Target", lift.getTargetPosition());
            telemetry.addData("Lift Position", lift.getCurrentPosition());
            telemetry.addData("Lift Error", lift.getError());
            telemetry.addData("At Target", lift.atTarget());
            telemetry.addData("Encoder Calibrated", lift.isHomed());
            telemetry.addData("PHYSICAL HOME SWITCH", lift.isHomeLimitPressed());
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

            if (lift.isHomeLimitPressed()) {
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
