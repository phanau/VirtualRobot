package virtual_robot.hardware;

import com.studiohartman.jamepad.ControllerManager;
import com.studiohartman.jamepad.ControllerState;

/**
 * Represents the Gamepad.
 *
 * Note: the fields in the class are all public, but they should not be changed from the OpMode code.
 * Note: in the original code from Git this was spelled "GamePad" but that's not what the real FTC code is so I've changed it.
 */
public class Gamepad {

    public volatile boolean x = false;
    public volatile boolean y = false;
    public volatile boolean a = false;
    public volatile boolean b = false;
    public volatile float left_stick_x  = 0;
    public volatile float left_stick_y = 0;
    public volatile float right_stick_x = 0;
    public volatile float right_stick_y = 0;
    public volatile boolean dpad_up = false;
    public volatile boolean dpad_down = false;
    public volatile boolean dpad_left = false;
    public volatile boolean dpad_right = false;
    public volatile boolean back = false;
    public volatile boolean guide = false;
    public volatile boolean start = false;
    public volatile boolean left_bumper = false;
    public volatile boolean right_bumper = false;
    public volatile boolean left_stick_button = false;
    public volatile boolean right_stick_button = false;
    public volatile float left_trigger = 0;
    public volatile float right_trigger = 0;

    private ControllerManager controllers = new ControllerManager();

    public Gamepad(){
        controllers.initSDLGamepad();
        update();
    }

    public void update(){
        ControllerState state = controllers.getState(0);
        x = state.x;
        y = state.y;
        a = state.a;
        b = state.b;
        left_stick_x = state.leftStickX;
        left_stick_y = -state.leftStickY;
        right_stick_x = state.rightStickX;
        right_stick_y = -state.rightStickY;
        dpad_up = state.dpadUp;
        dpad_down = state.dpadDown;
        dpad_left = state.dpadLeft;
        dpad_right = state.dpadRight;
        back = state.back;
        guide = state.guide;
        start = state.start;
        left_bumper = state.lb;
        right_bumper = state.rb;
        left_stick_button = state.leftStickClick;
        right_stick_button = state.rightStickClick;
        left_trigger = state.leftTrigger;
        right_trigger = state.rightTrigger;
    }

    public void release(){
        controllers.quitSDLGamepad();
    }

    /**
     * Display a summary of this gamepad, including the state of all buttons, analog sticks, and triggers
     * @return a summary
     */
    @Override
    public String toString() {
        String buttons = new String();
        if (dpad_up) buttons += "dpad_up ";
        if (dpad_down) buttons += "dpad_down ";
        if (dpad_left) buttons += "dpad_left ";
        if (dpad_right) buttons += "dpad_right ";
        if (a) buttons += "a ";
        if (b) buttons += "b ";
        if (x) buttons += "x ";
        if (y) buttons += "y ";
        if (guide) buttons += "guide ";
        if (start) buttons += "start ";
        if (back) buttons += "back ";
        if (left_bumper) buttons += "left_bumper ";
        if (right_bumper) buttons += "right_bumper ";
        if (left_stick_button) buttons += "left stick button ";
        if (right_stick_button) buttons += "right stick button ";

        return String.format("\nlx: % 1.2f ly: % 1.2f \nrx: % 1.2f ry: % 1.2f \nlt: %1.2f rt: %1.2f %s",
                left_stick_x, left_stick_y,
                right_stick_x, right_stick_y, left_trigger, right_trigger, buttons);
    }

}

