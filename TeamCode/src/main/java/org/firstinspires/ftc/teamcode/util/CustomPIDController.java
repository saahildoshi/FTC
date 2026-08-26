package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class CustomPIDController {

    private double kP;
    private double kI;
    private double kD;

    private double target;

    private double integralSum = 0;
    private double lastError = 0;

    private double integralLimit = 500;

    private final ElapsedTime timer = new ElapsedTime();

    public CustomPIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        timer.reset();
    }

    public double calculate(double currentPosition) {

        double error = target - currentPosition;

        double deltaTime = timer.seconds();
        timer.reset();

        // Protect against extremely small loop times
        if (deltaTime <= 0) {
            deltaTime = 0.001;
        }

        // Proportional
        double proportional = kP * error;

        // Integral
        integralSum += error * deltaTime;

        // Anti-windup
        integralSum = Math.max(
                -integralLimit,
                Math.min(integralLimit, integralSum)
        );

        double integral = kI * integralSum;

        // Derivative
        double derivative = (error - lastError) / deltaTime;
        double derivativeOutput = kD * derivative;

        lastError = error;

        return proportional + integral + derivativeOutput;
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public double getTarget() {
        return target;
    }

    public double getError(double currentPosition) {
        return target - currentPosition;
    }

    public void setPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
        timer.reset();
    }
}
