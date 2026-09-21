/*
Fixed Auto:
1. The robot sets target to the cell it thinks is 'UP'
2. scans AprilTag
3. corrects until perfect alignment
4. anchors its location
5. fires (Ben)

Alternate:
1. The robot sets target to the cell it thinks is 'UP'
2. scans AprilTag, if failed, switches status of hive and back to step 1.
3. corrects until perfect alignment
4. anchors its location
5. fires (Ben)
6. If shot twice with the same state of the cell, auto flip (efficiency)
*/

package org.firstinspires.ftc.teamcode.subsystems.camera;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

import java.util.List;


public class ShootingSequence {

    // How close is "perfect enough"
    private static final double X_TOLERANCE = 0.0001;      // inches
    private static final double Y_TOLERANCE = 0.0001;      // inches
    private static final double YAW_TOLERANCE = 0.001;    // degrees


    /*
     * Returns true if alignment and anchoring succeeded.
     * Returns false if no AprilTag cluster could be read.
     *
     * anchorPose is the known true Road Runner pose of the robot
     * when it is perfectly aligned for this shot.
     */
    public static boolean alignAndAnchor(
            MecanumDrive drive,
            AprilTagReader aprilTagReader,
            Pose2d anchorPose) {

        while (true) {

            List<AprilTagReader.Reading> readings =
                    aprilTagReader.getReadings();

            // Higher-level autonomous code decides what to do if this happens.
            if (readings.isEmpty()) {
                return false;
            }

            // For now, use the visible cluster.
            AprilTagReader.Reading reading = readings.get(0);


            // Check whether the camera already sees the desired alignment.
            double xError =
                    reading.x;

            double yError =
                    reading.y - 30.0;

            double yawError =
                    reading.yaw;


            if (Math.abs(xError) <= X_TOLERANCE &&
                    Math.abs(yError) <= Y_TOLERANCE &&
                    Math.abs(yawError) <= YAW_TOLERANCE) {

                // Robot is physically at the known anchor position.
                drive.localizer.setPose(anchorPose);

                return true;
            }


            // Use the AprilTag reading to calculate the next Road Runner target.
            AprilTagTargetCorrection.TargetPose correctedTarget =
                    AprilTagTargetCorrection.getNewTarget(
                            drive.localizer.getPose().position.x,
                            drive.localizer.getPose().position.y,
                            drive.localizer.getPose().heading.toDouble(),
                            reading
                    );


            // Drive to the corrected target.
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .strafeToLinearHeading(
                                    new Vector2d(
                                            correctedTarget.x,
                                            correctedTarget.y
                                    ),
                                    correctedTarget.yaw
                            )
                            .build()
            );

            // Loop again:
            // read tag again -> correct again -> repeat until aligned.
        }
    }
}