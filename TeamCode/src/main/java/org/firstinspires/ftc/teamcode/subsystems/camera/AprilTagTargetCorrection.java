package org.firstinspires.ftc.teamcode.subsystems.camera;

public class AprilTagTargetCorrection {

    private static final double OPTIMAL_SHOOTING_X = 0.0;   // inches
    private static final double OPTIMAL_SHOOTING_Y = 30.0;  // inches
    private static final double OPTIMAL_SHOOTING_YAW = 0.0; // degrees


    public static TargetPose getNewTarget(
            double targetX,
            double targetY,
            double targetYaw,
            AprilTagReader.Reading reading) {

        // Read only the three values needed from AprilTagReader
        double cameraX = reading.x;
        double cameraY = reading.y;
        double cameraYaw = reading.yaw;


        double xError =
                cameraX - OPTIMAL_SHOOTING_X;

        double yError =
                cameraY - OPTIMAL_SHOOTING_Y;

        double yawError =
                cameraYaw - OPTIMAL_SHOOTING_YAW;


        double newTargetX =
                targetX + xError;

        double newTargetY =
                targetY + yError;

        double newTargetYaw =
                targetYaw + Math.toRadians(yawError);


        return new TargetPose(
                newTargetX,
                newTargetY,
                newTargetYaw
        );
    }


    public static class TargetPose {

        public final double x;
        public final double y;
        public final double yaw;

        public TargetPose(double x, double y, double yaw) {
            this.x = x;
            this.y = y;
            this.yaw = yaw;
        }
    }
}