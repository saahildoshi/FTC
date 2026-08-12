package org.firstinspires.ftc.teamcode.util;

/**
 * BUTTON EDGE-DETECTION HELPER
 *
 * FTC gamepad booleans remain true for every loop while a button is held.
 * Sometimes we only want ONE event per press (toggle field-centric mode, open a
 * claw once, select a preset once, etc.).
 *
 * wasPressed() returns true only on the transition from released -> pressed.
 */
public final class Button {
    private boolean previous;

    public boolean wasPressed(boolean current) {
        boolean pressedThisLoop = current && !previous;
        previous = current;
        return pressedThisLoop;
    }
}
