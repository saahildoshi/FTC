# Optional Mechanism Scaffolding

This branch adds **examples only** for mechanisms that are not established by the current robot code.

Nothing in `MechanismOptions.java` or `MechanismControlOptions.java` is active robot behavior. The mechanism implementations and control snippets are intentionally commented out.

## Proposed hardware-map names

These names are placeholders and can be changed before anything is enabled:

| Mechanism | Proposed device | Proposed configuration name |
|---|---|---|
| Lift | DC motor | `leftLift` |
| Lift | DC motor | `rightLift` |
| Arm/pivot | DC motor | `arm` |
| Intake option 1 | DC motor | `intake` |
| Claw | positional servo | `claw` |
| Intake option 2 | CR servo | `intakeServo` |

## Included options

### Two-motor lift

Includes encoder presets, `RUN_TO_POSITION`, manual control, brake behavior, and a maximum encoder limit. All encoder positions are fabricated starting points and must be measured on the actual robot.

### Single-motor arm/pivot

Includes stow, intake, score, and maximum encoder positions plus manual control. The positions and power values are placeholders.

### Motor intake

Simple intake/outtake/stop implementation for a DC motor.

### Servo claw

Simple open/close implementation. The example positions (`0.70` and `0.30`) must not be trusted until the physical linkage is checked.

### CR-servo intake

Alternative intake implementation if the team chooses a continuous-rotation servo rather than a DC motor.

## Suggested Gamepad 2 concept

A commented control layout is included in `teleop/MechanismControlOptions.java`:

- Left stick: manual lift
- Right stick: manual arm
- D-pad: lift presets
- Triggers: intake/outtake
- Bumpers: claw open/close

The existing drivetrain controls are not modified.

## Before enabling a mechanism

1. Decide which physical mechanism is actually being built.
2. Confirm motor/servo model and hub port.
3. Set the Robot Controller configuration name.
4. Confirm motor and encoder direction.
5. Establish physical hard stops.
6. Measure safe encoder or servo limits.
7. Reduce initial test power.
8. Test the subsystem by itself.
9. Only then integrate it into `TeleOpDrive` and autonomous actions.
