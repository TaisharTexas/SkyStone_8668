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

import java.lang.reflect.Array;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

@TeleOp(name="SBF Teleop", group="Amazing")

public class SBF_Teleop extends OpMode
{
    Pursuit pursuit = new Pursuit(0, 0, telemetry);
    Robot robot = new Robot();
    SbfJoystick customPad1 = new SbfJoystick();
    SbfJoystick customPad2 = new SbfJoystick();
    CameraVision camera = new CameraVision();

    double shoulderPos;
    double wristPos;

    int currentXEncoder = 0;
    int currentYEncoder = 0;
    double currentHeading = 0;

    double loopTime=0;


    public void init()
    {
        robot.init(telemetry, hardwareMap, false, 0.0);
        customPad1.init(telemetry, hardwareMap, gamepad1);
        customPad2.init(telemetry, hardwareMap, gamepad2);

        robot.releaseFoundation();
        shoulderPos = .89;
//        lift.horizontal.setPosition(.7);
        wristPos = 0.135;


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

        telemetry.addData("horiz", robot.lift.horizontal.getPosition());
        telemetry.addData("d up", customPad2.getDpadUp());
        telemetry.addData("d down", customPad2.getDpadDown());



        /*Chassis control */
        robot.joystickDrive(customPad1.getLeftStickX(),
                            customPad1.getLeftStickY(),
                            customPad1.getRightStickX(),
                            customPad1.getRightStickY(),
                            afterburners());

//        lift controls
        robot.lift.verticalDrive(customPad2.getRightStickY()*.75);
        telemetry.addData("right vertical: ", robot.lift.rightVertical.getCurrentPosition());
        telemetry.addData("left vertical: ", robot.lift.leftVertical.getCurrentPosition());

        double[] shoulderVals = {.86, .7, .62};
        if(customPad2.getDpadDown())
        {
            shoulderPos += .01;
//            if(shoulderPos==shoulderVals[1])
//            {
//                shoulderPos=shoulderVals[0];
//            }
//            else if(shoulderPos==shoulderVals[2])
//            {
//                shoulderPos=shoulderVals[1];
//            }
//            else
//            {
//                shoulderPos=shoulderVals[0];
//            }
        }
        else if(customPad2.getDpadUp())
        {
//            if(shoulderPos==shoulderVals[0])
//            {
//                shoulderPos=shoulderVals[1];
//            }
//            else if(shoulderPos==shoulderVals[1])
//            {
//                shoulderPos=shoulderVals[2];
//            }
//            else
//            {
//                shoulderPos=shoulderVals[2];
//            }
            shoulderPos -= .01;
        }
        shoulderPos = Range.clip(shoulderPos, .62, .89);
        robot.lift.horizontalDrive(shoulderPos);



        //intake controls
        if(customPad1.getLeftTrigger() != 0)
        {
            robot.intakeIn(customPad1.getLeftTrigger());
        }
        else if(customPad1.getRightTrigger() != 0)
        {
            robot.intakeOut(customPad1.getRightTrigger());
        }
        else
        {
            robot.intakeStop();
        }

        if(customPad1.getLeftBumper())
        {
            robot.servosIn();
        }

        //foundation grabber controls
        if(customPad1.getX())
        {
            robot.grabFoundation();
        }
        else if(customPad1.getY())
        {
            robot.releaseFoundation();
        }

        //claw controls
        if(customPad2.getA())
        {
            robot.lift.grabClaw();
        }
        else if(customPad2.getY())
        {
            robot.lift.releaseClaw();
        }

        //wrist controls
        if(customPad2.getX())
        {
            robot.lift.wristDeploy();
            wristPos = 0.525;
        }
        else if(customPad2.getB())
        {
            robot.lift.wristRetract();
            wristPos = 0.135;
        }
        else if(customPad2.getLeftBumper())
        {
            wristPos += 0.01;
            wristPos = Range.clip( wristPos, 0.135, 0.9);
            robot.lift.wristDrive(wristPos);
        }
        else if ( customPad2.getRightBumper())
        {
            wristPos -= 0.01;
            wristPos = Range.clip( wristPos, 0.135, 0.9);
            robot.lift.wristDrive(wristPos);
        }

        if(customPad1.getB())
        {
            robot.lift.goHome();
            shoulderPos = .89;
        }

//        if(customPad2.getLeftTrigger() > .1)
//        {
//            robot.lift.autoExtend();
//            shoulderPos = .62;
//        }
//        if(customPad2.getRightTrigger() > .1)
//        {
//            robot.lift.goHome();
//            shoulderPos = .89;
//        }

        getRuntime();


        loopTime = getRuntime();
        resetStartTime();
        telemetry.addData("Loop Time ", "%.3f", loopTime);
        telemetry.addData("Gamepad left stick x, Left Stick X", "%.3f, %.3f", gamepad1.left_stick_x, customPad1.getLeftStickX());
        telemetry.addData("Gamepad left stick y, Left Stick Y", "%.3f, %.3f", gamepad1.left_stick_y, customPad1.getLeftStickY());
        telemetry.addData("Gamepad right stick x: ", "%.3f", customPad1.getRightStickX());
        telemetry.addData("Heading ", "%.3f", robot.getHeading());
//        telemetry.addData("Robot Location: ", "%.3f","%.3f", robot.);

//        telemetry.addData("SS Position: ", robot.eyeOfSauron.getSkyStonePosition());

    }

    public double afterburners()
    {
        double maximumSpeed;

        if(customPad1.getRightBumper())
        {
            maximumSpeed = .9;
        }
        else
        {
            maximumSpeed = .55;
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