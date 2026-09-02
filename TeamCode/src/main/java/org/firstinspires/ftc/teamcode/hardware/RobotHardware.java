package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.acmerobotics.roadrunner.ftc.LazyHardwareMapImu;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.config.RobotConstants;

public final class RobotHardware {
    public DcMotorEx leftFront;
    public DcMotorEx leftBack;
    public DcMotorEx rightBack;
    public DcMotorEx rightFront;
    public IMU imu;

    public LazyImu lazyImu;

    public Encoder par;
    public Encoder perp;
    public VoltageSensor voltageSensor;

    public DcMotorEx leftLift;
    public TouchSensor magneticLimitSwitch;
    public Servo claw;
    public DcMotorEx intake;

    public void initVerifiedHardware(HardwareMap hardwareMap) {
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

        par = new OverflowEncoder(new RawEncoder(rightBack));
        perp = new OverflowEncoder(new RawEncoder(leftBack));

        par.setDirection(DcMotorSimple.Direction.REVERSE);
        perp.setDirection(DcMotorSimple.Direction.REVERSE);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        leftLift = hardwareMap.get(
                DcMotorEx.class,
                RobotConstants.HardwareNames.LIFT);

        magneticLimitSwitch = hardwareMap.get(
                TouchSensor.class,
                RobotConstants.HardwareNames.MAGNETIC_LIMIT_SWITCH);

        claw = hardwareMap.get(
                Servo.class,
                RobotConstants.HardwareNames.CLAW);

        intake = hardwareMap.get(
                DcMotorEx.class,
                RobotConstants.HardwareNames.INTAKE);
    }

    public void stopDriveMotors() {
        if (leftFront != null) leftFront.setPower(0);
        if (leftBack != null) leftBack.setPower(0);
        if (rightBack != null) rightBack.setPower(0);
        if (rightFront != null) rightFront.setPower(0);
    }

    public void stopLiftMotor() {
        if (leftLift != null) leftLift.setPower(0);
    }

    public void stopIntakeMotor() {
        if (intake != null) intake.setPower(0);
    }
}
