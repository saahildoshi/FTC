package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;

import com.acmerobotics.roadrunner.ftc.LazyHardwareMapImu;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.config.RobotConstants;

/**
 * HARDWARE MAPPING LAYER
 *
 * RobotHardware is the bridge between Java and the physical Control/Expansion
 * Hub. It asks FTC's HardwareMap for each configured device by name.
 *
 * WHY USE THIS CLASS:
 * An OpMode should describe robot behavior, not repeatedly look up motors.
 * Centralizing hardware makes configuration mistakes easier to find and gives
 * every subsystem the same view of the robot.
 *
 * IMPORTANT:
 * The existing Road Runner MecanumDrive still owns/configures the drivetrain
 * during normal driving. These references are primarily useful for hardware
 * checks, diagnostics, and future subsystem organization. We intentionally do
 * not set drive motor direction here because doing so could conflict with the
 * team's already-tuned MecanumDrive configuration.
 */
public final class RobotHardware {
    // ----------------------------------------------------------------------
    // ACTIVE / VERIFIED DRIVETRAIN HARDWARE
    // ----------------------------------------------------------------------
    public DcMotorEx leftFront;
    public DcMotorEx leftBack;
    public DcMotorEx rightBack;
    public DcMotorEx rightFront;
    public IMU imu;

    public LazyImu lazyImu;

    public Encoder par;
    public Encoder perp;
    public VoltageSensor voltageSensor;

    // ----------------------------------------------------------------------
    // ACTIVE MECHANISM HARDWARE
    // ----------------------------------------------------------------------

    /**
     * Single lift motor selected from the earlier optional two-motor design.
     * The Robot Controller configuration name is "leftLift".
     */
    public DcMotorEx leftLift;

    /**
     * Positional servo that opens and closes the claw.
     * The Robot Controller configuration name is "claw".
     */
    public Servo claw;

    /**
     * Connect Java fields to the physical devices configured in the FTC Robot
     * Controller. If one of these names does not exist, initialization will fail
     * immediately so the team knows the hardware configuration needs attention.
     */
    public void initVerifiedHardware(HardwareMap hardwareMap) {
        // Four mecanum drive motors used by the existing Road Runner drivetrain.
        leftFront = hardwareMap.get(
                DcMotorEx.class,
                RobotConstants.HardwareNames.LEFT_FRONT);

        leftBack = hardwareMap.get(
                DcMotorEx.class,
                RobotConstants.HardwareNames.LEFT_BACK);

        rightBack = hardwareMap.get(
                DcMotorEx.class,
                RobotConstants.HardwareNames.RIGHT_BACK);

        rightFront = hardwareMap.get(
                DcMotorEx.class,
                RobotConstants.HardwareNames.RIGHT_FRONT);

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(
                IMU.class,
                RobotConstants.HardwareNames.IMU);

        lazyImu = new LazyHardwareMapImu(
                hardwareMap,
                RobotConstants.HardwareNames.IMU,
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

// Dead-wheel encoders
        par = new OverflowEncoder(
                new RawEncoder(rightBack));

        perp = new OverflowEncoder(
                new RawEncoder(leftBack));

        //par.setDirection(DcMotorSimple.Direction.REVERSE);

        //perp.setDirection(DcMotorSimple.Direction.REVERSE);

// Battery voltage
        voltageSensor =
                hardwareMap.voltageSensor.iterator().next();

// Mechanisms
        leftLift = hardwareMap.get(
                DcMotorEx.class,
                RobotConstants.HardwareNames.LIFT);

        claw = hardwareMap.get(
                Servo.class,
                RobotConstants.HardwareNames.CLAW);

    }

    /**
     * Immediately removes drivetrain power. This is useful for tests and as an
     * emergency-safe cleanup method. Road Runner normally commands these motors.
     */
    public void stopDriveMotors() {
        if (leftFront != null) leftFront.setPower(0);
        if (leftBack != null) leftBack.setPower(0);
        if (rightBack != null) rightBack.setPower(0);
        if (rightFront != null) rightFront.setPower(0);
    }

    /** Stop the active lift motor if it has been initialized. */
    public void stopLiftMotor() {
        if (leftLift != null) leftLift.setPower(0);
    }

    /* ======================================================================
       OPTIONAL MECHANISM HARDWARE -- INTENTIONALLY DISABLED

       When the team selects actual mechanisms, uncomment ONLY the devices that
       physically exist, add their imports, and map them using the names in
       RobotConstants.OptionalMechanisms.

       Example declarations:

       public DcMotorEx leftLift;
       public DcMotorEx rightLift;
       public DcMotorEx arm;
       public DcMotorEx intake;
       public Servo claw;
       public CRServo intakeServo;

       Example initialization:

       leftLift = hardwareMap.get(DcMotorEx.class,
               RobotConstants.OptionalMechanisms.LEFT_LIFT);
       rightLift = hardwareMap.get(DcMotorEx.class,
               RobotConstants.OptionalMechanisms.RIGHT_LIFT);
       arm = hardwareMap.get(DcMotorEx.class,
               RobotConstants.OptionalMechanisms.ARM);
       intake = hardwareMap.get(DcMotorEx.class,
               RobotConstants.OptionalMechanisms.INTAKE);
       claw = hardwareMap.get(Servo.class,
               RobotConstants.OptionalMechanisms.CLAW);
       intakeServo = hardwareMap.get(CRServo.class,
               RobotConstants.OptionalMechanisms.CR_SERVO_INTAKE);
       ====================================================================== */
}
