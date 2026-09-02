package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

/**
 * Simple intake motor subsystem.
 *
 * Positive power runs the intake forward and negative power runs it backward.
 * The subsystem always clips commands to the legal FTC motor range of -1.0 to 1.0.
 */
public final class IntakeSubsystem {

    private final DcMotorEx intakeMotor;

    public IntakeSubsystem(RobotHardware robot) {
        intakeMotor = robot.intake;
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPower(double power) {
        intakeMotor.setPower(Range.clip(power, -1.0, 1.0));
    }

    public void forward(double power) {
        setPower(Math.abs(power));
    }

    public void reverse(double power) {
        setPower(-Math.abs(power));
    }

    public void stop() {
        intakeMotor.setPower(0.0);
    }

    public double getPower() {
        return intakeMotor.getPower();
    }

    public int getCurrentPosition() {
        return intakeMotor.getCurrentPosition();
    }

    public double getVelocity() {
        return intakeMotor.getVelocity();
    }
}
