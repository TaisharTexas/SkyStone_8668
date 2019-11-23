package org.firstinspires.ftc.teamcode.sbfHardware;

import com.qualcomm.hardware.logitech.LogitechGamepadF310;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 *
 *
 * @author Andrew, SBF Robotics
 * */
public class SbfJoystick
{
    Telemetry telemetry;
    HardwareMap hardwareMap;

    Gamepad myGamepad;
    //Gamepad gamepad2;

//    public void init(Telemetry telem, HardwareMap hwMap, Gamepad theGamepad1, Gamepad theGamepad2)
    public void init(Telemetry telem, HardwareMap hwMap, Gamepad theGamepad )
    {
        hardwareMap = hwMap;
        telemetry = telem;
        myGamepad = theGamepad;
       // gamepad2 = theGamepad2;

    }

    //GAMEPAD 1
    public double getLeftStickX()
    {
        telemetry.addData("1, left stick x: ", myGamepad.left_stick_x);
        return myGamepad.left_stick_x;
    }

    public double getLeftStickY()
    {
        telemetry.addData("1, left stick y: ", myGamepad.left_stick_y);
        return myGamepad.left_stick_y;
    }

    public double getRightStickX()
    {
        telemetry.addData("1, right stick x: ", myGamepad.right_stick_x);
        return myGamepad.right_stick_x;
    }

    public double getRightStickY()
    {
        telemetry.addData("1, right stick y: ", myGamepad.right_stick_y);
        return myGamepad.right_stick_y;
    }

    public boolean getRightBumper()
    {
        return myGamepad.right_bumper;
    }

    public boolean getX()
    {
        return myGamepad.x;
    }

    public boolean getY()
    {
        return myGamepad.y;
    }


    // GAMEPAD 2
//    public double get2RightStickY()
//    {
//        telemetry.addData("2, right stick y: ", gamepad2.right_stick_y);
//        return gamepad2.right_stick_y;
//    }

    public double getLeftTrigger()
    {
        return myGamepad.left_trigger;
    }

    public double getRightTrigger()
    {
        return myGamepad.right_trigger;
    }

    public boolean getDpadDown()
    {
        return myGamepad.dpad_down;
    }

    public boolean getDpadUp()
    {
        return myGamepad.dpad_up;
    }

    public boolean getA()
    {
        return myGamepad.a;
    }
    public boolean getB()
    {
        return myGamepad.b;
    }
//    public boolean getX()
//    {
//        return gamepad2.x;
//    }
//    public boolean get2y()
//    {
//        return  gamepad2.y;
//    }

}
