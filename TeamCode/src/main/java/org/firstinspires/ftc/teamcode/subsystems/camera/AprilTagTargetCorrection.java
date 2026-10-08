package org.firstinspires.ftc.teamcode.subsystems.camera;

public class AprilTagTargetCorrection {

    /*
     * Anchor position magnitudes relative to field center.
     *
     * Field center = (0, 0)
     *
     * X:
     *     LEFT  = red  = negative
     *     RIGHT = blue = positive
     *
     * Y:
     *     FRONT = far side      = positive
     *     BACK  = audience side = negative
     *
     * Put the actual anchor distances here later.
     */
    private static final double ANCHOR_X = 0.0;   // inches
    private static final double ANCHOR_Y = 30.0;   // inches
    private static final double ANCHOR_YAW = 0.0; // radians


    public enum LeftRight {
        LEFT,   // red
        RIGHT   // blue
    }


    public enum FrontBack {
        FRONT,  // far side
        BACK    // audience side
    }


    public static TargetPose getNewTarget(
            double targetX,
            double targetY,
            double targetYaw,
            AprilTagReader.Reading reading,
            LeftRight leftRight,
            FrontBack frontBack) {


        // Read only the three values needed from AprilTagReader.
        double cameraX =
                reading.x;

        double cameraY =
                reading.y;

        double cameraYaw =
                reading.yaw;


        /*
         * Desired camera alignment:
         *
         * x = 0 inches
         * y = 30 inches
         * yaw = 0 degrees
         */
        double xError =
                cameraX;

        double yError =
                cameraY-30;

        double yawError =
                cameraYaw;


        // =========================================================
        // FIELD DIRECTION
        // =========================================================

        double xSign;

        if (leftRight == LeftRight.LEFT) {

            // Red side
            xSign = -1.0;

        } else {

            // Blue side
            xSign = 1.0;
        }


        double ySign;

        if (frontBack == FrontBack.FRONT) {

            // Far side
            ySign = 1.0;

        } else {

            // Audience side
            ySign = -1.0;
        }


        // =========================================================
        // CORRECTED ROAD RUNNER TARGET
        // =========================================================

        double newTargetX =
                targetX
                        + xSign * xError;

        double newTargetY =
                targetY
                        + ySign * yError;


        /*
         * Leave yaw logic as it was for now.
         * Its quadrant-dependent sign can be added later.
         */
        double newTargetYaw =
                targetYaw
                        + Math.toRadians(yawError);


        return new TargetPose(
                newTargetX,
                newTargetY,
                newTargetYaw
        );
    }


    // =============================================================
    // ANCHOR POSE
    // =============================================================

    public static AnchorPose getAnchorPose(
            LeftRight leftRight,
            FrontBack frontBack) {


        double xSign;

        if (leftRight == LeftRight.LEFT) {

            // Red
            xSign = -1.0;

        } else {

            // Blue
            xSign = 1.0;
        }


        double ySign;

        if (frontBack == FrontBack.FRONT) {

            // Far side
            ySign = 1.0;

        } else {

            // Audience side
            ySign = -1.0;
        }


        double anchorX =
                xSign * ANCHOR_X;

        double anchorY =
                ySign * ANCHOR_Y;


        return new AnchorPose(
                anchorX,
                anchorY,
                ANCHOR_YAW
        );
    }


    // =============================================================
    // TARGET POSE
    // =============================================================

    public static class TargetPose {

        public final double x;
        public final double y;
        public final double yaw;


        public TargetPose(
                double x,
                double y,
                double yaw) {

            this.x = x;
            this.y = y;
            this.yaw = yaw;
        }
    }


    // =============================================================
    // ANCHOR POSE
    // =============================================================

    public static class AnchorPose {

        public final double x;
        public final double y;
        public final double yaw;


        public AnchorPose(
                double x,
                double y,
                double yaw) {

            this.x = x;
            this.y = y;
            this.yaw = yaw;
        }
    }
}