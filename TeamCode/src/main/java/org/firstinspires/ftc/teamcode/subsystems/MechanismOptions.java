package org.firstinspires.ftc.teamcode.subsystems;

/**
 * OPTIONAL / FABRICATED MECHANISM SCAFFOLDING
 *
 * Nothing in this file controls the robot. The examples below are intentionally
 * commented out because the current repository does not establish the actual
 * lift, arm, intake, or claw hardware.
 *
 * Use this as a menu of possible implementations. Before enabling anything:
 *  1. Choose the actual mechanism design.
 *  2. Match every hardware-map name to the Robot Controller configuration.
 *  3. Confirm motor direction and encoder direction.
 *  4. Establish safe encoder limits and servo positions on the real robot.
 *  5. Test at low power with the robot supported safely.
 */
public final class MechanismOptions {
    private MechanismOptions() { }

    /* ======================================================================
       OPTION A: TWO-MOTOR ENCODER LIFT
       ======================================================================

    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.DcMotorEx;
    import com.qualcomm.robotcore.hardware.DcMotorSimple;
    import com.qualcomm.robotcore.hardware.HardwareMap;

    public static final class Lift {
        private final DcMotorEx leftLift;
        private final DcMotorEx rightLift;

        // FABRICATED starting points -- MUST be measured/tuned.
        public static final int HOME_TICKS = 0;
        public static final int LOW_TICKS = 500;
        public static final int MID_TICKS = 1100;
        public static final int HIGH_TICKS = 1800;
        public static final int MAX_TICKS = 2100;
        public static final double MOVE_POWER = 0.75;

        public Lift(HardwareMap hardwareMap) {
            leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
            rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");

            rightLift.setDirection(DcMotorSimple.Direction.REVERSE);
            leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public void moveTo(int requestedTicks) {
            int target = Math.max(HOME_TICKS, Math.min(MAX_TICKS, requestedTicks));
            leftLift.setTargetPosition(target);
            rightLift.setTargetPosition(target);
            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftLift.setPower(MOVE_POWER);
            rightLift.setPower(MOVE_POWER);
        }

        public void manual(double power) {
            leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftLift.setPower(power);
            rightLift.setPower(power);
        }

        public void stop() {
            leftLift.setPower(0);
            rightLift.setPower(0);
        }
    }
    */

    /* ======================================================================
       OPTION B: ENCODER ARM / PIVOT
       ======================================================================

    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.DcMotorEx;
    import com.qualcomm.robotcore.hardware.HardwareMap;

    public static final class Arm {
        private final DcMotorEx armMotor;

        // FABRICATED positions -- determine from the actual arm geometry.
        public static final int STOW_TICKS = 0;
        public static final int INTAKE_TICKS = 250;
        public static final int SCORE_TICKS = 900;
        public static final int MAX_TICKS = 1200;
        public static final double MOVE_POWER = 0.55;

        public Arm(HardwareMap hardwareMap) {
            armMotor = hardwareMap.get(DcMotorEx.class, "arm");
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public void moveTo(int requestedTicks) {
            int target = Math.max(STOW_TICKS, Math.min(MAX_TICKS, requestedTicks));
            armMotor.setTargetPosition(target);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(MOVE_POWER);
        }

        public void manual(double power) {
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotor.setPower(power);
        }

        public void stop() {
            armMotor.setPower(0);
        }
    }
    */

    /* ======================================================================
       OPTION C: MOTOR-DRIVEN INTAKE
       ======================================================================

    import com.qualcomm.robotcore.hardware.DcMotorEx;
    import com.qualcomm.robotcore.hardware.HardwareMap;

    public static final class Intake {
        private final DcMotorEx intakeMotor;

        // FABRICATED power values -- adjust for the selected mechanism.
        public static final double INTAKE_POWER = 1.0;
        public static final double OUTTAKE_POWER = -0.75;

        public Intake(HardwareMap hardwareMap) {
            intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        }

        public void intake() {
            intakeMotor.setPower(INTAKE_POWER);
        }

        public void outtake() {
            intakeMotor.setPower(OUTTAKE_POWER);
        }

        public void stop() {
            intakeMotor.setPower(0);
        }
    }
    */

    /* ======================================================================
       OPTION D: SERVO CLAW
       ======================================================================

    import com.qualcomm.robotcore.hardware.HardwareMap;
    import com.qualcomm.robotcore.hardware.Servo;

    public static final class Claw {
        private final Servo clawServo;

        // FABRICATED positions -- find safe values on the physical mechanism.
        public static final double OPEN_POSITION = 0.70;
        public static final double CLOSED_POSITION = 0.30;

        public Claw(HardwareMap hardwareMap) {
            clawServo = hardwareMap.get(Servo.class, "claw");
        }

        public void open() {
            clawServo.setPosition(OPEN_POSITION);
        }

        public void close() {
            clawServo.setPosition(CLOSED_POSITION);
        }
    }
    */

    /* ======================================================================
       OPTION E: CR-SERVO INTAKE
       Use this instead of Option C if the intake is continuous-rotation servo.
       ======================================================================

    import com.qualcomm.robotcore.hardware.CRServo;
    import com.qualcomm.robotcore.hardware.HardwareMap;

    public static final class CRServoIntake {
        private final CRServo intakeServo;

        public CRServoIntake(HardwareMap hardwareMap) {
            intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        }

        public void intake() {
            intakeServo.setPower(1.0);
        }

        public void outtake() {
            intakeServo.setPower(-1.0);
        }

        public void stop() {
            intakeServo.setPower(0.0);
        }
    }
    */
}
