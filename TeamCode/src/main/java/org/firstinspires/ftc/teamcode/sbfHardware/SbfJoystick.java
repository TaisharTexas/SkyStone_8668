package org.firstinspires.ftc.teamcode.sbfHardware;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * A custom gamepad hardware class that allows for the gamepad outputs to be modified when needed.
 *
 * @author Andrew, 8668 Should Be Fine!
 * */
public class SbfJoystick
{
    /** A telemetry object passed down from the opmode. */
    Telemetry telemetry;
    /** A hardware map object passed down from the opmode. */
    HardwareMap hardwareMap;
    /** A gamepad hardware object. */
    Gamepad myGamepad;

    /**
     * Runs once -- initializes all the needed hardware.
     * @param telem  A telemetry object passed down from the opmode.
     * @param hwMap  A hardware map object passed down from the opmode.
     * @param theGamepad  A gamepad object passed down from the opmode.
     */
    public void init(Telemetry telem, HardwareMap hwMap, Gamepad theGamepad )
    {
        hardwareMap = hwMap;
        telemetry = telem;
        myGamepad = theGamepad;
    }

    //GAMEPAD 1

    /**
     * Gets the left stick x value.
     * @return the left stick x value.
     */
    public double getLeftStickX()
    {
//        telemetry.addData("1, left stick x: ", myGamepad.left_stick_x);
        return myGamepad.left_stick_x;
    }

    /**
     * Gets the left stick y value.
     * @return the left stick y value.
     */
    public double getLeftStickY()
    {
//        telemetry.addData("1, left stick y: ", myGamepad.left_stick_y);
        return -myGamepad.left_stick_y;
    }

    /**
     * Gets the right stick x value.
     * @return the right stick x value.
     */
    public double getRightStickX()
    {
//        telemetry.addData("1, right stick x: ", myGamepad.right_stick_x);
        return myGamepad.right_stick_x;
    }

    /**
     * Gets the right stick y value.
     * @return the right stick y value.
     */
    public double getRightStickY()
    {
//        telemetry.addData("1, right stick y: ", myGamepad.right_stick_y);
        return myGamepad.right_stick_y;
    }

    /**
     * Gets the right bumper value.
     * @return Whether or not the bumper is pressed.
     */
    public boolean getRightBumper()
    {
        return myGamepad.right_bumper;
    }

    /**
     * Gets the left bumper value.
     * @return Whether or not the bumper is pressed.
     */
    public boolean getLeftBumper()
    {
        return myGamepad.left_bumper;
    }

    /**
     * Gets the x button value.
     * @return Whether or not the x button is pressed.
     */
    public boolean getX()
    {
        return myGamepad.x;
    }

    /**
     * Get the y button value.
     * @return Whether or not the y button is pressed.
     */
    public boolean getY()
    {
        return myGamepad.y;
    }
    /**
     * Gets the a button value.
     * @return Whether or not the a button is being pressed.
     */
    public boolean getA()
    {
        return myGamepad.a;
    }

    /**
     * Gets the b button value.
     * @return Whether or not the b button is being pressed.
     */
    public boolean getB()
    {
        return myGamepad.b;
    }

    /**
     * Get the left trigger value.
     * @return the value of the left trigger.
     */
    public double getLeftTrigger()
    {
        return myGamepad.left_trigger;
    }

    /**
     * Gets the right trigger value.
     * @return the right trigger value.
     */
    public double getRightTrigger()
    {
        return myGamepad.right_trigger;
    }

    /**
     * Gets the dpad value for down.
     * @return Whether or not the dpad is being pressed down.
     */
    public boolean getDpadDown()
    {
        return myGamepad.dpad_down;
    }

    /**
     * Gets the dpad value for up.
     * @return Whether or not the dpad is being pressed up.
     */
    public boolean getDpadUp()
    {
        return myGamepad.dpad_up;
    }

    /**
     * Gets the dpad value for left.
     * @return Whether or not the dpad is being pressed left.
     */
    public boolean getDpadLeft()
    {
        return myGamepad.dpad_left;
    }

    /**
     * Gets the dpad value for right.
     * @return Whether or not the dpad is being pressed right.
     */
    public boolean getDpadRight()
    {
        return myGamepad.dpad_right;
    }

}
