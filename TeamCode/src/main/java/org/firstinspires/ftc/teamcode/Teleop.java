package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.os.Environment;
import android.os.storage.StorageManager;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.sbfUtil.DataLogger;

import java.lang.reflect.Array;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

@TeleOp(name="teleop", group="pure")

public class Teleop extends OpMode
{
    Pursuit pursuit = new Pursuit(0, 0, telemetry);
    Robot robot = new Robot();


    /* Chassis Control */
    /** The x-axis of the left joystick on the gamepad. Used for chassis control*/
    double lStickX;
    /** The x-axis of the right joystick on the gamepad. Used for chassis control*/
    double rStickX;
    /** The y-axis of the left joystick on the gamepad. Used for chassis control*/
    double lStickY;
    /** The y-axis of the right joystick on the gamepad. Used for chassis control*/
    double rStickY;


    int currentXEncoder = 0;
    int currentYEncoder = 0;
    double currentHeading = 0;


    double loopTime=0;

    private DataLogger myData;


    public void init()
    {
        robot.init(telemetry, hardwareMap, true);

        myData = new DataLogger("8668_Robot_Data");
        myData.addField("elapsedTime");
        myData.addField("xEncoderPos");
        myData.addField("yEncoderPos");
        myData.newLine();

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
    }

    public void loop()
    {
        robot.update();

        /* Chassis Control */
        /** The x-axis of the left joystick on the gamepad. Used for chassis control*/
        lStickX = -gamepad1.left_stick_x;
        /** The x-axis of the right joystick on the gamepad. Used for chassis control*/
        rStickX = -gamepad1.right_stick_x;
        /** The y-axis of the left joystick on the gamepad. Used for chassis control*/
        lStickY = gamepad1.left_stick_y;
        /** The y-axis of the right joystick on the gamepad. Used for chassis control*/
        rStickY = gamepad1.right_stick_y;

        /* Tell the robot to move */
        robot.joystickDrive(lStickX, lStickY, rStickX, rStickY, afterburners());

        myData.addField(loopTime);
        myData.addField(currentXEncoder);
        myData.addField(currentYEncoder);
        myData.newLine();

        telemetry.addData("camera", robot.eyeOfSauron.tensorFlowMineralDetection());

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
