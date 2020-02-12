package org.firstinspires.ftc.teamcode.sbfHardware;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.Locale;
import java.util.concurrent.TimeUnit;

public class Intake
{
    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    /**
     * Intake Items
     */
    public ExpansionHubMotor leftIntake = null;
    public ExpansionHubMotor rightIntake = null;
    private CRServo leftInSupport = null;
    private CRServo rightInSupport = null;
//    private ColorSensor rampSignalC = null;
    private DistanceSensor rampSignalD = null;
    private DistanceSensor backSignal = null;
//    private CRServo rightInSupport2 = null;
//    private CRServo leftInSupport2 = null;
    private double stallCurrent = 5100;
    private static double leftMaxIntakeSpd = 0.95;
    private static double rightMaxIntakeSpd = 0.9;
    private static double leftMaxIntakeSpdAuto = 1;
    private static double rightMaxIntakeSpdAuto = .9;
    private double stoneDistanceFront = 6;
    private double stoneDistanceBack = 6;

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;

    private int relativeLayoutId;
    private View relativeLayout;

    public void init(Telemetry telem, HardwareMap hwmap)
    {
        telemetry = telem;
        hardwareMap = hwmap;

        try
        {
            leftIntake = hardwareMap.get(ExpansionHubMotor.class, "yEncoder");
            leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        catch (Exception p_exception)
        {
            telemetry.addData("left intake not found in config file","");
            leftIntake = null;
        }
        try
        {
            rightIntake = hardwareMap.get(ExpansionHubMotor.class, "xEncoder");
            rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        catch (Exception p_exception)
        {
            telemetry.addData("right intake not found in config file", "");
            rightIntake = null;
        }

        try
        {
            leftInSupport = hardwareMap.get(CRServo.class, "leftIn");
            leftInSupport.setDirection(CRServo.Direction.REVERSE);
        }
        catch (Exception p_exeception)
        {
            telemetry.addData("leftIn not found in config file", 0);
            leftInSupport = null;
        }
        try
        {
            rightInSupport = hardwareMap.get(CRServo.class, "rightIn");
            rightInSupport.setDirection(CRServo.Direction.REVERSE);
        }
        catch (Exception p_exeception)
        {
            telemetry.addData("rightIn not found in config file", 0);
            rightInSupport = null;
        }
//        try
//        {
//            rightInSupport2 = hardwareMap.get(CRServo.class, "rightIn2");
//            rightInSupport2.setDirection(CRServo.Direction.REVERSE);
//        }
//        catch (Exception p_exeception)
//        {
//            telemetry.addData("rightIn2 not found in config file", 0);
//            rightInSupport2 = null;
//        }
//        try
//        {
//            leftInSupport2 = hardwareMap.get(CRServo.class, "leftIn2");
//            leftInSupport2.setDirection(CRServo.Direction.REVERSE);
//        }
//        catch (Exception p_exeception)
//        {
//            telemetry.addData("leftIn2 not found in config file", 0);
//            leftInSupport2 = null;
//        }

//        try
//        {
//            rampSignalC = hardwareMap.get(ColorSensor.class, "rampSignal");
//        }
//        catch (Exception p_exception)
//        {
//            telemetry.addData("rampSignalC not found in config file", 0);
//            rampSignalC = null;
//        }
        try
        {
            rampSignalD = hardwareMap.get(DistanceSensor.class, "rampSignal");
        }
        catch (Exception p_exception)
        {
            telemetry.addData("rampSignalD not found in config file", 0);
            rampSignalD = null;
        }
        try
        {
            backSignal = hardwareMap.get(DistanceSensor.class, "backSignal");
        }
        catch (Exception p_exception)
        {
            telemetry.addData("backSignal not found in config file", 0);
            backSignal = null;
        }

    }

    /**
     * Intake - Rotate the intake wheels to reverse a stone out of the intake.
     * @param power
     */
    public void intakeOut(double power)
    {
        intakeDrive(-power);
    }

    public boolean rampSignal()
    {
        // send the info back to driver station using telemetry function.
        telemetry.addData("Distance (cm)", String.format(Locale.US, "%.02f", rampSignalD.getDistance(DistanceUnit.CM)));

        if(!(Double.isNaN(rampSignalD.getDistance(DistanceUnit.CM))))
        {
            if(rampSignalD.getDistance(DistanceUnit.CM) <= stoneDistanceFront)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }



    }

    public boolean backSignal()
    {
        // send the info back to driver station using telemetry function.
        telemetry.addData("Distance (cm)", String.format(Locale.US, "%.02f", backSignal.getDistance(DistanceUnit.CM)));

        if(!(Double.isNaN(backSignal.getDistance(DistanceUnit.CM))))
        {
            if(backSignal.getDistance(DistanceUnit.CM) <= stoneDistanceBack)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }

    }

    /**
     * Intake - Rotate the intake wheels to take in a stone into the intake.
     * @param power  the power input from the gamepad
     */
    public void intakeIn(double power)
    {
        intakeDrive(power);
    }

    public void servosDrive(double power)
    {
        leftInSupport.setPower(-power);
//        leftInSupport2.setPower(-power);
        rightInSupport.setPower(power);
//        leftInSupport2.setPower(power);
    }

//    private boolean isLeftStalled()
//    {
//
//        if(leftIntake.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.MILLIAMPS) > stallCurrent)
//        {
//            return true;
//        }
//        else
//        {
//            return false;
//        }
//    }
//
//    private boolean isRightStalled()
//    {
//
//        if(rightIntake.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.MILLIAMPS) > stallCurrent)
//        {
//            return true;
//        }
//        else
//        {
//            return false;
//        }
//    }

    /**
     * Intake - stop the intake wheels
     */
    public void intakeStop()
    {
//        xEncoder.setPower(0.0);
        if(leftIntake != null)
        {
            leftIntake.setPower(0.0);
            leftInSupport.setPower(0.0);
//            leftInSupport2.setPower(0.0);

        }
        else
        {
            telemetry.addData("left intake is null", "cannot use");
        }
//        yEncoder.setPower(0.0);
        if(rightIntake != null)
        {
            rightIntake.setPower(0.0);
            rightInSupport.setPower(0.0);
//            rightInSupport2.setPower(0.0);
        }
        else
        {
            telemetry.addData("right intake is null", "cannot use");
        }

    }

    /**
     * Drives intake motors & servos.
     * @param power  the power at which the motors drive
     *               - power = eject
     *               + power = intake
     * */
    public void intakeDrive(double power)
    {
        leftIntake.setPower(-power*leftMaxIntakeSpdAuto);
        rightIntake.setPower(-power*rightMaxIntakeSpdAuto);
        servosDrive(power);

    }

    public void sensorDone()
    {
        // Set the panel back to the default color
        relativeLayout.post(new Runnable()
        {
            public void run()
            {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });
    }


}

