package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class CustomePIDController {
    private double kP;
    private double kI;
    private double kD;

    private double target;

    private double integralSum = 0;
    private double lastError = 0;

    private double integralLimit = 500;

    private final ElapsedTime timer = new ElapsedTime();

    public CustomePIDController(double kP, double kI, double kD){
        this.kP=kP;
        this.kI=kI;
        this.kD=kD;

        timer.reset();
    }

    public double calculate(double currentPosition) {
        double error = target - currentPosition;
        double deltaTime = timer.seconds();
        timer.reset();

        if (deltaTime <= 0) {
            deltaTime = 0.001;
        }

        double proportional = kP * error;
        integralSum += error * deltaTime;

        integralSum = Math.max(-integralLimit, Math.min(integralLimit, integralSum));

        double integral = kI * integralSum;

        double derivative = (error - lastError) / deltaTime;
        double derivativeOutput = kD * derivative;

        lastError = error;

        return proportional + integral + derivativeOutput;
    }

    public void setTarget(double target){
        this.target=target;
    }

    public double getTarget(){
        return target;
    }

    public void setPID(double kP, double kI, double kD){
        this.kP = kP;
        this.kI=kI;
        this.kD=kD;
    }

    public void reset(){
        integralSum = 0;
        lastError = 0;
        timer.reset();
    }
}
