# FTC Robot Code Architecture

This document explains how the team's Java code maps to the physical robot.

## 1. `config/` — facts and adjustable values

`RobotConstants.java` is the central configuration reference.

Think of it as the software version of a robot wiring/configuration sheet. Hardware-map names must match the Robot Controller configuration exactly. Driver constants affect robot feel. Optional mechanism names are kept separate because those mechanisms have not yet been selected.

## 2. `hardware/` — Java-to-robot connection

`RobotHardware.java` asks FTC's `HardwareMap` for physical devices.

A hardware-map name such as `leftFront` is not a variable invented by Java; it must match the device name configured in the Robot Controller app on the Control Hub/Driver Station system.

The active Road Runner drive still owns drivetrain configuration during driving so this layer does not overwrite tuned motor directions.

## 3. `drive/` — tuned Road Runner drivetrain

The existing `drive/MecanumDrive.java` is the low-level Road Runner drivetrain implementation. It contains robot-specific tuning that should not be replaced with generic values.

It connects Road Runner's desired robot motion to the four mecanum motors and the localizer.

`TwoDeadWheelLocalizer.java` estimates robot movement from the team's odometry/encoder arrangement.

## 4. `subsystems/` — one class per physical robot function

`DriveSubsystem.java` wraps the existing Road Runner drive with simpler commands.

Future physical mechanisms should each become their own subsystem:

- LiftSubsystem
- ArmSubsystem
- IntakeSubsystem
- ClawSubsystem

`MechanismOptions.java` currently contains disabled examples for those choices. They are not active because the physical mechanisms are not yet verified.

## 5. `actions/` — reusable autonomous behaviors

An Action should represent a small behavior such as:

- drive to a pose
- raise lift
- move arm
- open claw
- run intake

Small Actions can be combined with Road Runner `SequentialAction` and `ParallelAction`.

`StopDriveAction.java` is an intentionally simple active example of the Road Runner Action pattern.

## 6. `teleop/` — driver decisions

The existing `TeleOpDrive.java` remains the reference/competition driver code because it already contains tuned features.

`ModularTeleOpExample.java` explains how to migrate that behavior toward the subsystem architecture without deleting working features.

`MechanismControlOptions.java` contains disabled Gamepad 2 ideas for mechanisms that have not been selected.

## 7. `autonomous/` — match strategy

Autonomous OpModes should be high-level recipes. They should create subsystems and then combine Road Runner drive trajectories with mechanism Actions.

`ModularAutoExample.java` is intentionally inactive until the team knows the 2026-27 starting pose, field geometry, and mechanisms.

## 8. `testing/` and `tests/` — prove one thing at a time

Testing OpModes isolate one feature. A mechanism should work reliably in its own test before it is added to TeleOp or autonomous.

`ModularHardwareCheck.java` verifies the known drivetrain hardware names without moving the robot.

## 9. `util/` — reusable helpers

Utility classes solve small repeated programming problems.

- `Button.java` detects a single new button press.
- `RobotTelemetry.java` standardizes important Driver Station telemetry.

## Recommended development sequence

1. Confirm Android Studio builds with zero errors.
2. Run hardware-name checks.
3. Verify drivetrain direction.
4. Verify IMU orientation.
5. Verify Road Runner localization.
6. Preserve/recheck Road Runner tuning.
7. Select one physical mechanism.
8. Enable only that subsystem's hardware.
9. Create a dedicated mechanism test OpMode.
10. Measure limits/presets.
11. Add the mechanism to TeleOp.
12. Create reusable Actions.
13. Combine Actions in autonomous.

## Core rule for students

**Do not make five systems work at once. Make one layer work, test it, document it, and then connect it to the next layer.**
