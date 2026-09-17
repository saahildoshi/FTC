package org.firstinspires.ftc.teamcode.subsystems.camera;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;


public class AprilTagReader {

    private final AprilTagProcessor aprilTagProcessor;
    private final VisionPortal visionPortal;


    public AprilTagReader(HardwareMap hardwareMap) {

        // FTC's AprilTag processor.
        // Uses the current-game AprilTag library by default.
        aprilTagProcessor =
                AprilTagProcessor.easyCreateWithDefaults();

        // Connect the processor to the C270 webcam.
        visionPortal =
                new VisionPortal.Builder()
                        .setCamera(
                                hardwareMap.get(
                                        WebcamName.class,
                                        "Webcam 1"
                                )
                        )
                        .setCameraResolution(
                                new Size(640, 480)
                        )
                        .addProcessor(aprilTagProcessor)
                        .build();
    }


    /*
     * Returns every AprilTag cluster currently visible.
     */
    public List<Reading> getReadings() {

        List<Reading> readings = new ArrayList<>();

        List<AprilTagDetection> detections =
                aprilTagProcessor.getDetections();

        for (AprilTagDetection detection : detections) {

            // We only care about cluster detections here.
            if (detection instanceof AprilTagClusterDetection) {

                AprilTagClusterDetection cluster =
                        (AprilTagClusterDetection) detection;

                if (detection.ftcPose != null) {

                    readings.add(
                            new Reading(
                                    cluster.metadata.name,

                                    detection.ftcPose.x,
                                    detection.ftcPose.y,
                                    detection.ftcPose.z,

                                    detection.ftcPose.yaw,
                                    detection.ftcPose.pitch,
                                    detection.ftcPose.roll,

                                    detection.ftcPose.range,
                                    detection.ftcPose.bearing,
                                    detection.ftcPose.elevation,

                                    cluster.percentClusterFound
                            )
                    );
                }
            }
        }

        return readings;
    }


    /*
     * Call when you are completely finished using the camera.
     */
    public void close() {
        visionPortal.close();
    }


    /*
     * Stores one cluster reading.
     */
    public static class Reading {

        public final String clusterName;

        // inches
        public final double x;
        public final double y;
        public final double z;

        // degrees
        public final double yaw;
        public final double pitch;
        public final double roll;

        // inches / degrees
        public final double range;
        public final double bearing;
        public final double elevation;

        public final int percentClusterFound;


        public Reading(
                String clusterName,

                double x,
                double y,
                double z,

                double yaw,
                double pitch,
                double roll,

                double range,
                double bearing,
                double elevation,

                int percentClusterFound) {

            this.clusterName = clusterName;

            this.x = x;
            this.y = y;
            this.z = z;

            this.yaw = yaw;
            this.pitch = pitch;
            this.roll = roll;

            this.range = range;
            this.bearing = bearing;
            this.elevation = elevation;

            this.percentClusterFound = percentClusterFound;
        }
    }
}