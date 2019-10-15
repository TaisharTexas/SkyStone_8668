package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.sbfUtil.DataLogger;

@TeleOp(name="teleop", group="pure")

public class Teleop extends OpMode
{
    Pursuit pursuit = new Pursuit(0, 0, telemetry);
    Robot robot = new Robot();

    /* Chassis Control */
    /** The x-axis of the left joystick on the gamepad. Used for chassis control*/
    double lStickX = -gamepad1.left_stick_x;
    /** The x-axis of the right joystick on the gamepad. Used for chassis control*/
    double rStickX = -gamepad1.right_stick_x;
    /** The y-axis of the left joystick on the gamepad. Used for chassis control*/
    double lStickY = gamepad1.left_stick_y;
    /** The y-axis of the right joystick on the gamepad. Used for chassis control*/
    double rStickY = gamepad1.right_stick_y;


    int currentXEncoder = 0;
    int currentYEncoder = 0;
    double currentHeading = 0;


    double loopTime=0;

    private DataLogger myData;


    public void init()
    {
        robot.init(telemetry, hardwareMap);

        myData = new DataLogger("8668_Robot_Data");
        myData.addField("elapsedTime");
        myData.addField("xEncoderPos");
        myData.addField("yEncoderPos");
        myData.newLine();

    }

    @Override
    public void init_loop()
    {
        telemetry.addData("heading: ", robot.getHeading());

        telemetry.addData("path: ", Environment.getExternalStorageDirectory().getPath() + "/");
    }

    public void start()
    {
        resetStartTime();
    }

    public void loop()
    {
        robot.update();

        /* Tell the robot to move */
        robot.joystickDrive(lStickX, lStickY, rStickX, rStickY, afterburners());

        myData.addField(loopTime);
        myData.addField(currentXEncoder);
        myData.addField(currentYEncoder);
        myData.newLine();

    }

    public double afterburners()
    {
        double maximumSpeed;

        if(gamepad1.right_bumper)
        {
            maximumSpeed = 1;
        }
        else
        {
            maximumSpeed = .5;
        }

        return maximumSpeed;
    }
}
