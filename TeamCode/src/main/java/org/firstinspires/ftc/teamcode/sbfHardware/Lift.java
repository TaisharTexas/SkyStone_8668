package org.firstinspires.ftc.teamcode.sbfHardware;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.revextensions2.ExpansionHubMotor;

public class Lift
{
    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    private RevTouchSensor touchL = null;
    private RevTouchSensor touchR = null;

    public ExpansionHubMotor leftVertical = null;
    public ExpansionHubMotor rightVertical = null;

    public Servo horizontal = null;
    public Servo claw = null;
    public Servo wrist = null;

    public int encoder = 0;

    double thePosition;


    public void init(Telemetry telem, HardwareMap hwmap)
    {
        telemetry = telem;
        hardwareMap = hwmap;

        try
        {
            leftVertical = (ExpansionHubMotor) hardwareMap.get(DcMotorEx .class, "leftV");
            leftVertical.setDirection(DcMotorEx.Direction.REVERSE);
            leftVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
        catch (Exception p_execption)
        {
            leftVertical = null;
            telemetry.addData("left vertical not found in config file", "");
        }

        try
        {
            rightVertical = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, "rightV");
            rightVertical.setDirection(DcMotorEx.Direction.FORWARD);
            rightVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        catch (Exception p_execption)
        {
            rightVertical = null;
            telemetry.addData("right vertical not found in config file", "");
        }




        touchL = hardwareMap.get(RevTouchSensor.class, "touchL");
        touchR = hardwareMap.get(RevTouchSensor.class, "touchR");

        try
        {
            horizontal = hardwareMap.get(Servo.class, "horizontal");
            horizontal.setDirection(Servo.Direction.FORWARD);
        }
        catch (Exception p_exeception)
        {
            telemetry.addData("horizontal not found in config file", 0);
            horizontal = null;
        }
        try
        {
            claw = hardwareMap.get(Servo.class, "claw");
            claw.setDirection(Servo.Direction.FORWARD);
        }
        catch (Exception p_exeception)
        {
            telemetry.addData("claw not found in config file", 0);
            claw = null;
        }
        try
        {
            wrist = hardwareMap.get(Servo.class, "wrist");
            wrist.setDirection(Servo.Direction.FORWARD);
        }
        catch (Exception p_exeception)
        {
            telemetry.addData("wrist not found in config file", 0);
            wrist = null;
        }

    }

    public void verticalDrive(double power)
    {
        if ( encoder > 4700 && power < 0 ) {
            leftVertical.setPower(0.0);
            rightVertical.setPower(0.0);
        }
        else if(touchR.isPressed() || touchL.isPressed())
        {
            leftVertical.setPower(0.2);
            rightVertical.setPower(0.2);
        }
        else if ( encoder < 400  && power > 0)
        {
            leftVertical.setPower(-0.15);
            rightVertical.setPower(-0.15);
        }

        else if(power > .01 || power < -.01)
        {
            leftVertical.setPower(-power);
            rightVertical.setPower(-power);
        }

        else
        {
            leftVertical.setPower(0.0);
            rightVertical.setPower(0.0);
        }


    }

    public void horizontalDrive(double position)
    {
        if(horizontal != null)
        {
            telemetry.addData("horizontal: ", horizontal.getPosition());
            horizontal.setPosition(position);
        }
    }

    public void grabClaw()
    {
        if(claw != null)
        {
            claw.setPosition(.9);
        }
    }
    public void releaseClaw()
    {
        if(claw != null)
        {
            claw.setPosition(.5);
        }
    }

    public void wristDeploy()
    {
        if(wrist != null)
        {
            wrist.setPosition(.9);
        }
    }
    public void wristRetract()
    {
        if(wrist != null)
        {
            wrist.setPosition(.135);
        }
    }
    public void wristDrive( double position )
    {
        if(horizontal != null)
        {
            wrist.setPosition(position);
        }
    }


}
