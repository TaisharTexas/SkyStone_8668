package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.os.Environment;
import android.os.storage.StorageManager;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.sbfHardware.CameraVision;
import org.firstinspires.ftc.teamcode.sbfHardware.Lift;
import org.firstinspires.ftc.teamcode.sbfHardware.Robot;
import org.firstinspires.ftc.teamcode.sbfHardware.SbfJoystick;
import org.firstinspires.ftc.teamcode.sbfUtil.DataLogger;

import java.lang.reflect.Array;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

@TeleOp(name="SBF Teleop", group="Amazing")

public class SBF_Teleop extends OpMode
{
    Pursuit pursuit = new Pursuit(0, 0, telemetry);
    Robot robot = new Robot();
    Lift lift = new Lift();
    SbfJoystick customPad = new SbfJoystick();
    CameraVision camera = new CameraVision();

    double shoulderPos;

    int currentXEncoder = 0;
    int currentYEncoder = 0;
    double currentHeading = 0;

    double loopTime=0;

    private DataLogger myData;


    public void init()
    {
        robot.init(telemetry, hardwareMap, false);
        lift.init(telemetry, hardwareMap);
        customPad.init(telemetry, hardwareMap, gamepad1, gamepad2);

        robot.releaseFoundation();
        shoulderPos = .99;
//        lift.horizontal.setPosition(.7);

//        myData = new DataLogger("8668_Robot_Data");
//        myData.addField("elapsedTime");
//        myData.addField("xEncoderPos");
//        myData.addField("yEncoderPos");
//        myData.newLine();

    }

    @Override
    public void stop()
    {
        robot.stop();
        super.stop();
    }

    @Override
    public void init_loop()
    {
        telemetry.addData("heading: ", robot.getHeading());


        telemetry.addData("path: ", Environment.getExternalStorageDirectory().getPath() + "/");
        telemetry.addData("path external: ", getExternalStoragePath(hardwareMap.appContext, true) );

    }

    public void start()
    {
        robot.start();
        resetStartTime();
//        lift.wrist.setPosition(.9);
//        lift.claw.setPosition(.1);
    }

    public void loop()
    {
        robot.update();

        telemetry.addData("horiz", lift.horizontal.getPosition());
        telemetry.addData("d up", customPad.get2DpadUp());
        telemetry.addData("d down", customPad.get2DpadDown());


        /*Chassis control */
        robot.joystickDrive(customPad.get1LeftStickX(),
                            customPad.get1LeftStickY(),
                            customPad.get1RightStickX(),
                            customPad.get1RightStickY(),
                            afterburners());

//        lift controls
        lift.verticalDrive(customPad.get2RightStickY()*.8);

        if(customPad.get2DpadDown())
        {
            shoulderPos += .005;
        }
        else if(customPad.get2DpadUp())
        {
            shoulderPos -= .005;
        }
        shoulderPos = Range.clip(shoulderPos, .65, .99);
        lift.horizontal.setPosition(shoulderPos);


        //intake controls
        if(customPad.get2LeftTrigger() != 0)
        {
            robot.intakeIn(customPad.get2LeftTrigger());
        }
        else if(customPad.get2RightTrigger() != 0)
        {
            robot.intakeOut(customPad.get2RightTrigger());
        }
        else
        {
            robot.intakeStop();
        }

        if(customPad.get1x())
        {
            robot.grabFoundation();
        }
        else if(customPad.get1y())
        {
            robot.releaseFoundation();
        }

        if(customPad.get2a())
        {
            lift.grabClaw();
        }
        else if(customPad.get2y())
        {
            lift.releaseClaw();
        }

        if(customPad.get2b())
        {
            lift.wristDeploy();
        }
        else if(customPad.get2x())
        {
            lift.wristRetract();
        }


//        myData.addField(loopTime);
//        myData.addField(xEncoderChange);
//        myData.addField(yEncoderChange);
//        myData.newLine();

        loopTime = getRuntime();
        resetStartTime();
        telemetry.addData("Loop Time ", "%.3f", loopTime);
        telemetry.addData("Gamepad left stick x, Left Stick X", "%.3f, %.3f", gamepad1.left_stick_x, customPad.get1LeftStickX());
        telemetry.addData("Gamepad left stick y, Left Stick Y", "%.3f, %.3f", gamepad1.left_stick_y, customPad.get1LeftStickY());
        telemetry.addData("Heading ", "%.3f", robot.currentHeading);
//        telemetry.addData("Robot Location: ", "%.3f","%.3f", robot.);

//        telemetry.addData("SS Position: ", robot.eyeOfSauron.getSkyStonePosition());

    }

    public double afterburners()
    {
        double maximumSpeed;

        if(customPad.get1RightBumper())
        {
            maximumSpeed = 1;
        }
        else
        {
            maximumSpeed = .5;
        }

        return maximumSpeed;
    }




    /**
     * Get external sd card path using reflection
     * @param mContext
     * @param is_removable is external storage removable
     * @return
     */
    private static String getExternalStoragePath(Context mContext, boolean is_removable) {

        StorageManager mStorageManager = (StorageManager) mContext.getSystemService(Context.STORAGE_SERVICE);
        Class<?> storageVolumeClazz = null;
        try {
            storageVolumeClazz = Class.forName("android.os.storage.StorageVolume");
            Method getVolumeList = mStorageManager.getClass().getMethod("getVolumeList");
            Method getPath = storageVolumeClazz.getMethod("getPath");
            Method isRemovable = storageVolumeClazz.getMethod("isRemovable");
            Object result = getVolumeList.invoke(mStorageManager);
            final int length = Array.getLength(result);
            for (int i = 0; i < length; i++) {
                Object storageVolumeElement = Array.get(result, i);
                String path = (String) getPath.invoke(storageVolumeElement);
                boolean removable = (Boolean) isRemovable.invoke(storageVolumeElement);
                if (is_removable == removable) {
                    return path;
                }
            }
        } catch (ClassNotFoundException e) {
            e.printStackTrace();
        } catch (InvocationTargetException e) {
            e.printStackTrace();
        } catch (NoSuchMethodException e) {
            e.printStackTrace();
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        }
        return null;
    }

}
