package org.firstinspires.ftc.teamcode.subsystems.camera;

public class ShootingSequence {

    /* Fixed Auto:
    1. The robot sets target to the cell it thinks is 'UP'
    2. scans AprilTag
    3. corrects until perfect alignment
    4. anchors its location
    5. fires (Ben)
    */

    /* Alternate:
    1. The robot sets target to the cell it thinks is 'UP'
    2. scans AprilTag, if failed, switches status of hive and back to step 1.
    3. corrects until perfect alignment
    4. anchors its location
    5. fires (Ben)
    6. If shot twice with the same state of the cell, auto flip (efficiency)
    */
}
