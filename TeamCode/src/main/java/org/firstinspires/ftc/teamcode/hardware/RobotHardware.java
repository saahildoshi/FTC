package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

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
    public DcMotorEx leftFront;
    public DcMotorEx leftBack;
    public DcMotorEx rightBack;
    public DcMotorEx rightFront;
    public IMU imu;

    /** Connect Java fields to devices configured on the Robot Controller. */
    public void initVerifiedHardware(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotorEx.class, RobotConstants.HardwareNames.LEFT_FRONT);
        leftBack = hardwareMap.get(DcMotorEx.class, RobotConstants.HardwareNames.LEFT_BACK);
        rightBack = hardwareMap.get(DcMotorEx.class, RobotConstants.HardwareNames.RIGHT_BACK);
        rightFront = hardwareMap.get(DcMotorEx.class, RobotConstants.HardwareNames.RIGHT_FRONT);
        imu = hardwareMap.get(IMU.class, RobotConstants.HardwareNames.IMU);
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
