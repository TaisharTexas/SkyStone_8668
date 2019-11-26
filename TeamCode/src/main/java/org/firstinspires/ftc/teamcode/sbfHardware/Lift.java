package org.firstinspires.ftc.teamcode.sbfHardware;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

    private ExpansionHubMotor leftVertical = null;
    private ExpansionHubMotor rightVertical = null;

    public Servo horizontal = null;
    public Servo claw = null;
    public Servo wrist = null;

    double thePosition;


    public void init(Telemetry telem, HardwareMap hwmap)
    {
        telemetry = telem;
        hardwareMap = hwmap;

        leftVertical = (ExpansionHubMotor) hardwareMap.get(DcMotorEx .class, "leftV");
        leftVertical.setDirection(DcMotorEx.Direction.FORWARD);

        rightVertical = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, "rightV");
        rightVertical.setDirection(DcMotorEx.Direction.FORWARD);

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
        if(touchR.isPressed() || touchL.isPressed())
        {
            leftVertical.setPower(-0.2);
            rightVertical.setPower(0.2);
        }
        else if(power > .01 || power < -.01)
        {
            leftVertical.setPower(power);
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


}
