package org.firstinspires.ftc.teamcode;

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

//    private Servo horizontal = null;
//    private Servo claw = null;
//    private Servo wrist = null;


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

//        try
//        {
//            horizontal = hardwareMap.get(Servo.class, "horizontal");
//        }
//        catch (Exception p_exeception)
//        {
//            telemetry.addData("horizontal not found in config file", 0);
//            horizontal = null;
//        }
//        try
//        {
//            claw = hardwareMap.get(Servo.class, "claw");
//        }
//        catch (Exception p_exeception)
//        {
//            telemetry.addData("claw not found in config file", 0);
//            claw = null;
//        }
//        try
//        {
//            wrist = hardwareMap.get(Servo.class, "wrist");
//        }
//        catch (Exception p_exeception)
//        {
//            telemetry.addData("wrist not found in config file", 0);
//            wrist = null;
//        }

    }

    public void verticalDrive(double power)
    {
        if(touchR.isPressed() || touchL.isPressed())
        {
            leftVertical.setPower(-0.2);
            rightVertical.setPower(0.2);
        }
        else if(power > .035 || power < -.035)
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

}
