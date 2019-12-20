package org.firstinspires.ftc.teamcode.Mentoring;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

//@TeleOp
/**
 * Written for Carl Bergman
 *
 * @author Andrew Lee, SBF Robotics
 *  */
@Disabled
public class Robot_Teleop extends OpMode
{

    public DcMotor left;
    public DcMotor right;
    public DcMotor techArm;
//    public DcMotor extend;
    public Servo elbow;
    public Servo fClaw;
    public Servo bClaw;

    double rStickY;
    double lStickY;
    double armRotate;

    public void init()
    {
        left = hardwareMap.get(DcMotor.class, "left");
        left.setDirection(DcMotor.Direction.FORWARD);

        right = hardwareMap.get(DcMotor.class, "right");
        right.setDirection(DcMotor.Direction.FORWARD);

        techArm = hardwareMap.get(DcMotor.class, "techArm");
        techArm.setDirection(DcMotor.Direction.FORWARD);

//        extend = hardwareMap.get(DcMotor.class, "extend");
//        extend.setDirection(DcMotor.Direction.FORWARD);

        elbow = hardwareMap.get(Servo.class, "elbow");
        fClaw = hardwareMap.get(Servo.class, "fClaw");
        bClaw = hardwareMap.get(Servo.class, "bClaw");




    }
    public void start()
    {

    }
    public void loop()
    {
        rStickY = gamepad1.right_stick_y;
        lStickY = gamepad1.left_stick_y;
        armRotate = gamepad2.left_stick_y;
        right.setPower(rStickY);
        left.setPower(lStickY);
        techArm.setPower(armRotate);

        if(gamepad2.a)
        {
            fClaw.setPosition(0.0);
            bClaw.setPosition(.9);

        }
        if(gamepad2.y)
        {
            fClaw.setPosition(.3);
            bClaw.setPosition(0.0);
        }

        if(gamepad2.x)
        {
            elbow.setPosition(0.3);
        }
        if(gamepad2.b)
        {
            elbow.setPosition(0.0);
        }


    }
    public void stop()
    {

        right.setPower(0.0);
        left.setPower(0.0);
    }
}
