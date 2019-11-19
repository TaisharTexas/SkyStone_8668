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

    Gamepad gamepad1;
    Gamepad gamepad2;

    public void init(Telemetry telem, HardwareMap hwMap, Gamepad theGamepad1, Gamepad theGamepad2)
    {
        hardwareMap = hwMap;
        telemetry = telem;
        gamepad1 = theGamepad1;
        gamepad2 = theGamepad2;

    }

    //GAMEPAD 1
    public double get1LeftStickX()
    {
        telemetry.addData("1, left stick x: ", gamepad1.left_stick_x);
        return gamepad1.left_stick_x;
    }

    public double get1LeftStickY()
    {
        telemetry.addData("1, left stick y: ", gamepad1.left_stick_y);
        return gamepad1.left_stick_y;
    }

    public double get1RightStickX()
    {
        telemetry.addData("1, right stick x: ", gamepad1.right_stick_x);
        return gamepad1.right_stick_x;
    }

    public double get1RightStickY()
    {
        telemetry.addData("1, right stick y: ", gamepad1.right_stick_y);
        return gamepad1.right_stick_y;
    }

    public boolean get1RightBumper()
    {
        return gamepad1.right_bumper;
    }

    public boolean get1x()
    {
        return gamepad1.x;
    }

    public boolean get1y()
    {
        return gamepad1.y;
    }


    // GAMEPAD 2
    public double get2RightStickY()
    {
        telemetry.addData("2, right stick y: ", gamepad2.right_stick_y);
        return gamepad2.right_stick_y;
    }

    public double get2LeftTrigger()
    {
        return gamepad2.left_trigger;
    }

    public double get2RightTrigger()
    {
        return gamepad2.right_trigger;
    }

    public boolean get2DpadDown()
    {
        return gamepad2.dpad_down;
    }

    public boolean get2DpadUp()
    {
        return gamepad2.dpad_up;
    }

    public boolean get2a()
    {
        return gamepad2.a;
    }
    public boolean get2b()
    {
        return gamepad2.b;
    }
    public boolean get2x()
    {
        return gamepad2.x;
    }
    public boolean get2y()
    {
        return  gamepad2.y;
    }

}
