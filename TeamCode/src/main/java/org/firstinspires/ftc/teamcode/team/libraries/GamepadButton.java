package org.firstinspires.ftc.teamcode.team.libraries;

import com.qualcomm.robotcore.hardware.Gamepad;
public class GamepadButton {
    public enum gamepadKeys {
        A,
        B,
        BACK,
        DPAD_DOWN,
        DPAD_LEFT,
        DPAD_RIGHT,
        DPAD_UP,
        GUIDE,
        LEFT_BUMPER,
        LEFT_STICK_BUTTON,
        RIGHT_BUMPER,
        RIGHT_STICK_BUTTON,
        START,
        X,
        Y,
        circle,
        square,
        triangle,
        cross,
        SHARE
    }
    private final Gamepad pad;
    private boolean previousState = false;
    private final gamepadKeys key;
    public GamepadButton(Gamepad pad, gamepadKeys key) {
        this.pad = pad;
        this.key = key;
    }

    public boolean isDown() {
        switch (key) {
            case A: return pad.a;
            case B: return pad.b;
            case BACK: return pad.back;
            case DPAD_DOWN: return pad.dpad_down;
            case DPAD_LEFT: return pad.dpad_left;
            case DPAD_UP: return pad.dpad_up;
            case DPAD_RIGHT: return pad.dpad_right;
            case GUIDE: return pad.guide;
            case LEFT_BUMPER: return pad.left_bumper;
            case LEFT_STICK_BUTTON: return pad.left_stick_button;
            case RIGHT_BUMPER: return pad.right_bumper;
            case RIGHT_STICK_BUTTON: return pad.right_stick_button;
            case START: return pad.start;
            case X: return pad.x;
            case Y: return pad.y;
            case circle: return pad.circle;
            case square: return pad.square;
            case triangle: return pad.triangle;
            case cross: return pad.cross;
            case SHARE: return pad.share;
            default: return false;
        }
    }

    public boolean isPressed() {
        boolean isPressed = (previousState != isDown()) && isDown();
        previousState = isDown();
        return isPressed;
    }

}
