package org.firstinspires.ftc.teamcode.sbfHardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.concurrent.TimeUnit;

public class Intake
{
    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    /**
     * Intake Items
     */
    private ExpansionHubMotor leftIntake = null;
    private ExpansionHubMotor rightIntake = null;
    private CRServo leftInSupport = null;
    private CRServo rightInSupport = null;
    private CRServo rightInSupport2 = null;
    private CRServo leftInSupport2 = null;
    private double stallCurrent = 5100;
    private static double leftMaxIntakeSpd = 0.6;
    private static double rightMaxIntakeSpd = 0.5;
    private static double leftMaxIntakeSpdAuto = .9;
    private static double rightMaxIntakeSpdAuto = .8;


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


    }

    /**
     * Intake - Rotate the intake wheels to reverse a stone out of the intake.
     * @param power
     */
    public void intakeOut(double power)
    {
        leftIntake.setPower(power * leftMaxIntakeSpd);
        leftInSupport.setPower(1);
//        leftInSupport2.setPower(1);
        rightIntake.setPower(-power * rightMaxIntakeSpd);
        rightInSupport.setPower(-1);
//        rightInSupport2.setPower(-1);

    }

    /**
     * Intake - Rotate the intake wheels to take in a stone into the intake.
     * @param power  the power input from the gamepad
     */
    public void intakeIn(double power)
    {

        // Adding these 2 variables so that we only access the expansion hub once per call.
        double leftIntakeCurrent = leftIntake.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.MILLIAMPS);
        double rightIntakeCurrent = rightIntake.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.MILLIAMPS);

        telemetry.addData("left intake milliamps: ", leftIntakeCurrent);
        telemetry.addData("right intake milliamps: ", rightIntakeCurrent);

        if( Math.abs(leftIntakeCurrent) > stallCurrent )  // can motor current be negative?
        {
            leftIntake.setPower(power * .75);
            leftInSupport.setPower(1);
            rightIntake.setPower(-power * .25);
            rightInSupport.setPower(-1);
        }
        else if( Math.abs(rightIntakeCurrent) > stallCurrent )
        {
            leftIntake.setPower(power * .25);
            leftInSupport.setPower(1);
            rightIntake.setPower(-power * .75);
            rightInSupport.setPower(-1);
        }
        else
        {
            leftIntake.setPower(-power * leftMaxIntakeSpd );
            leftInSupport.setPower(-1);
//            leftInSupport2.setPower(-1);
            rightIntake.setPower(power * rightMaxIntakeSpd);
            rightInSupport.setPower(1);
//            rightInSupport2.setPower(1);
        }

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

    public void intakeDrive(double power)
    {
        leftIntake.setPower(-power*leftMaxIntakeSpdAuto);
        rightIntake.setPower(power*rightMaxIntakeSpdAuto);
        servosDrive(power);

    }


}
