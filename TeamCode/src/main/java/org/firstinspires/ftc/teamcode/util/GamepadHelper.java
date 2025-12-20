package org.firstinspires.ftc.teamcode.util;

public class GamepadHelper {
    private boolean[] lastState = new boolean[20];
    private boolean[] toggleState = new boolean[20];

    public static final int A = 0;
    public static final int B = 1;
    public static final int X = 2;
    public static final int Y = 3;
    public static final int LB = 4;
    public static final int RB = 5;
    public static final int DPAD_UP = 6;
    public static final int DPAD_DOWN = 7;
    public static final int DPAD_LEFT = 8;
    public static final int DPAD_RIGHT = 9;
    public static final int LEFT_STICK_BUTTON = 10;
    public static final int RIGHT_STICK_BUTTON = 11;

    public boolean risingEdge(int button, boolean current) {
        boolean rising = current && !lastState[button];
        lastState[button] = current;
        return rising;
    }

    public boolean toggle(int button, boolean current) {
        if (current && !lastState[button]) {
            toggleState[button] = !toggleState[button];
        }
        lastState[button] = current;
        return toggleState[button];
    }

    public boolean getToggle(int button) {
        return toggleState[button];
    }

    public void setToggle(int button, boolean value) {
        toggleState[button] = value;
    }
}
