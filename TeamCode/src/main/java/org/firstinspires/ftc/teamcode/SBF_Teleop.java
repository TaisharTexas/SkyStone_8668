package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.os.Environment;
import android.os.storage.StorageManager;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.sbfHardware.CameraVision;
import org.firstinspires.ftc.teamcode.sbfHardware.Lift;
import org.firstinspires.ftc.teamcode.sbfHardware.Robot;
import org.firstinspires.ftc.teamcode.sbfHardware.SbfJoystick;
import org.firstinspires.ftc.teamcode.sbfHardware.StoneClaw;

import java.lang.reflect.Array;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.concurrent.TimeUnit;

@TeleOp(name="SBF Teleop", group="Amazing")
/**
 * Contains all the controls for operating the robot with gamepads.
 *
 * @author Andrew, 8668 Should Be Fine!
 * @see OpMode
 * */
public class SBF_Teleop extends OpMode
{
    /**
     * Sets the robot's starting position to 0,0 -- not used right now.
     */
    Pursuit pursuit = new Pursuit(0, 0, telemetry);
    /** Declares a robot object. */
    Robot robot = new Robot();
    /** Declares a custom gamepad1 object -- gamepad 1 controls the chassis and intake. */
    SbfJoystick customPad1 = new SbfJoystick();
    /** Declares a custom gamepad2 object -- gamepad 2 controls the lift and horizontal slides. */
    SbfJoystick customPad2 = new SbfJoystick();
    /** Declares a camera object -- not used right now. */
    CameraVision camera = new CameraVision();

    /** Limits how quickly the dpad on gamepad2 can be incremented. */
//    Deadline pad2Wait = new Deadline(175, TimeUnit.MILLISECONDS);
    /** A currently unused limit for gamepad 1. */
    Deadline pad1Wait = new Deadline( 200, TimeUnit.MILLISECONDS);

    /** The position the horizontal slide needs to be driven to (incremented up and down by the dpad on gamepad 2). */
    double shoulderPos;
    /** The position the wrist needs to be turned to. */
    double wristPos;
    Deadline capWait;

    double armPosition = .5;

    double clawPosition = .5;

    /** Stores the current x position of the robot. */
    int currentXEncoder = 0;
    /** Stores the current y position of the robot. */
    int currentYEncoder = 0;
    /** The current heading of the robot. */
    double currentHeading = 0;

    /** How long each iteration through loop() takes. */
    double loopTime=0;


    /** Runs once when INIT is pressed on the driver station.
     * Initializes all the robot hardware and both gamepads.
     * Sets all the servos to their starting positions.
     * */
    public void init()
    {
        robot.init(telemetry, hardwareMap, false, 0.0, true);
        customPad1.init(telemetry, hardwareMap, gamepad1);
        customPad2.init(telemetry, hardwareMap, gamepad2);

        robot.releaseFoundation();
//        shoulderPos = .29;
        robot.stoneClaw.stoneDrive(StoneClaw.stoneClawPositions.HOME);
        armPosition = .1;
        clawPosition = .28;
        wristPos = .05;
//        capWait = new Deadline(75, TimeUnit.SECONDS);


    }

    /** Runs once when STOP is pressed on the driver station.
     * Stops all motors on the robot and terminates any running programs/processes.
     */
    @Override
    public void stop()
    {
        robot.stopEverything();
        super.stop();
    }

    /** Runs repeatedly after init() is done but before START is pressed. */
    @Override
    public void init_loop()
    {

//        telemetry.addData("heading: ", robot.getRawHeading());

//        telemetry.addData("path: ", Environment.getExternalStorageDirectory().getPath() + "/");
//        telemetry.addData("path external: ", getExternalStoragePath(hardwareMap.appContext, true) );

    }

    /** Runs once when START is pressed on the driver station.
     * Calls the robot start and resets all the timers.
     * Upon completion initiates loop().*/
    public void start()
    {
        robot.start();
//        capWait.reset();
        resetStartTime();
        pad1Wait.reset();
//        pad2Wait.reset();
//        robot.lights.start();
//        lift.wrist.setPosition(.9);
//        lift.claw.setPosition(.1);
    }

    /** Runs repeatedly after start() until STOP is pressed on the driver station.
     * Contains all the control links between the gamepads and the robot. */
    public void loop()
    {
        robot.update();

//        telemetry.addData("horiz", robot.lift.horizontal.getPosition());
//        telemetry.addData("d up", customPad2.getDpadUp());
//        telemetry.addData("d down", customPad2.getDpadDown());

        telemetry.addData("stone claw", robot.stoneClaw.stoneClaw.getPosition());
        telemetry.addData("stone arm", robot.stoneClaw.stoneArm.getPosition());
        telemetry.addData("right touch", robot.lift.touchR.getValue());
        telemetry.addData("left touch", robot.lift.touchL.getValue());
        telemetry.addData("foundation left", robot.leftFoundation.getPosition());
        telemetry.addData("foundation right", robot.rightFoundation.getPosition());

        /*Chassis control */
        robot.joystickDrive(customPad1.getLeftStickX(),
                            customPad1.getLeftStickY(),
                            customPad1.getRightStickX(),
                            customPad1.getRightStickY(),
                            afterburners());

//        lift controls
        if(customPad2.getRightStickY() > .1 || customPad2.getRightStickY() < .1)
        {
            robot.isVLiftMoving = true;
        }
        else
        {
            robot.isVLiftMoving = false;
        }
        robot.lift.verticalDrive(customPad2.getRightStickY());
        telemetry.addData("vertical: ", robot.lift.vEncoder);

//        if(customPad2.getDpadDown())
//        {
//            robot.lift.horizontalDrive(-.6);
//        }
//        else if(customPad2.getDpadUp())
//        {
//            robot.lift.horizontalDrive(.6);
//        }
//        else
//        {
//            robot.lift.horizontalDrive(0.0);
//        }
//        if(robot.lift.hEncoder > 2000)
//        {
//            robot.intakeState=0;
//        }
//        telemetry.addData("horizontal pwr", robot.lift.vexHoriz.getPower());
//        telemetry.addData("horizontal pos", robot.lift.hEncoder);


        telemetry.addData("position: ", robot.location);

        //intake controls
        if(customPad1.getLeftTrigger() != 0)
        {
            robot.intakeInWithLights(customPad1.getLeftTrigger());
        }
        else if(customPad1.getRightTrigger() != 0)
        {
            robot.intake.intakeOut(customPad1.getRightTrigger()*0.6, true, true);
        }
        else
        {
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
            robot.intake.intakeStop();
        }
        if(customPad1.getDpadUp())
        {
            armPosition += .05;
        }
        else if(customPad1.getDpadDown())
        {
            armPosition -= .05;
        }
        if(customPad1.getDpadLeft())
        {
            clawPosition += .05;
        }
        else if(customPad1.getDpadRight())
        {
            clawPosition -= .05;
        }
        armPosition = Range.clip(armPosition,0,1);
        clawPosition = Range.clip(clawPosition,.28,.86);
        if(robot.stoneClaw.stoneArm!=null)
        {
            robot.stoneClaw.stoneArm.setPosition(armPosition);
            telemetry.addData("arm position: ", robot.stoneClaw.stoneArm.getPosition());

        }
        if(robot.stoneClaw.stoneClaw!=null)
        {
            robot.stoneClaw.stoneClaw.setPosition(clawPosition);
            telemetry.addData("claw position: ", robot.stoneClaw.stoneClaw.getPosition());

        }



//        if(customPad2.getLeftStickY()<-0.6)
//        {
////            if(capWait.hasExpired())
////            {
////                robot.lift.releaseCap();
////            }
//                robot.lift.releaseCap();
//
//        }
//        else if(customPad2.getLeftStickY()>.6)
//        {
////            if(capWait.hasExpired())
////            {
////                robot.lift.capStone.setPosition(.6);
////            }
//               robot.lift.capStone.setPosition(.6);
//
//        }

//        telemetry.addData("vex H#: ", robot.lift.vexHoriz.getPortNumber());
//        telemetry.addData("vex H: ", robot.lift.vexHoriz.getPower());

//        telemetry.addData("left y: ", customPad2.getLeftStickY());
//        telemetry.addData("cap stone: ", robot.lift.capStone.getPosition());

//        if(customPad1.getLeftBumper())
//        {
//            robot.intake.servosDrive(1);
//        }

//        telemetry.addData("ramp signal: ", robot.intake.rampSignal());
//        telemetry.addData("back signal: ", robot.intake.backSignal());

//        foundation grabber controls
        if(customPad1.getX())
        {
//            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            robot.grabFoundation();
        }
        else if(customPad1.getY())
        {
//            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
            robot.releaseFoundation();
        }

//        telemetry.addData("claw", robot.lift.claw.getPosition());
        //claw controls
//        if(customPad2.getA())
//        {
//            robot.lift.grabClaw();
//        }
//        else if(customPad2.getY())
//        {
//            robot.lift.releaseClaw();
//        }

//        telemetry.addData("wrist", robot.lift.wrist.getPosition());
        //wrist controls
//        if(customPad2.getX())
//        {
//            robot.lift.wristDeploy();
//            wristPos = 0.4;
//        }
//        else if(customPad2.getB())
//        {
//            robot.lift.wristRetract();
//            wristPos = 0.05;
//        }
//        else if(customPad2.getLeftTrigger()>.001)
//        {
//            wristPos += 0.01;
//            wristPos = Range.clip( wristPos, 0.01, 0.9);
//            robot.lift.wristDrive(wristPos);
//        }
//        else if ( customPad2.getRightTrigger()>.001)
//        {
//            wristPos -= 0.01;
//            wristPos = Range.clip( wristPos, 0.01, 0.9);
//            robot.lift.wristDrive(wristPos);
//        }

//        if(customPad1.getB())
//        {
//            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
//        }
//        else
//        {
//            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
//        }

//        if(customPad1.getB())
//        {
//            robot.lift.goHome();
//            shoulderPos = .89;
//        }

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
//        telemetry.addData("Loop Time ", "%.3f", loopTime);
//        telemetry.addData("Gamepad left stick x, Left Stick X", "%.3f, %.3f", gamepad1.left_stick_x, customPad1.getLeftStickX());
//        telemetry.addData("Gamepad left stick y, Left Stick Y", "%.3f, %.3f", gamepad1.left_stick_y, customPad1.getLeftStickY());
//        telemetry.addData("Gamepad right stick x: ", "%.3f", customPad1.getRightStickX());
//        telemetry.addData("Heading ", "%.3f", robot.getRawHeading());
//        telemetry.addData("Robot Location: ", "%.3f","%.3f", robot.);

//        telemetry.addData("SS Position: ", robot.eyeOfSauron.getSkyStonePosition());

    }

    /**
     * Sets the maximum speed for the chassis --
     * default is forty-five percent, afterburners is one-hundred percent.
     * @return The max speed.
     */
    public double afterburners()
    {
        double maximumSpeed;

        if(customPad1.getRightBumper())
        {
            maximumSpeed = 1;
        }
        else
        {
            maximumSpeed = .45;
        }

        return maximumSpeed;
    }




    /**
     * Get external sd card path using reflection
     * @param mContext Talks to the android device
     * @param is_removable is external storage removable
     * @return returns the path to the sd card, if found.
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
