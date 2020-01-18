package org.firstinspires.ftc.teamcode.Mentoring;
//package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

//@TeleOp
@Disabled
public class CarlTeleop extends OpMode
{
    //Drive motors
    public DcMotor frontRight;
    public DcMotor backRight;
    public DcMotor frontLeft;
    public DcMotor backLeft;

    public DcMotor techArm;
    public DcMotor extend;
    public Servo elbow;
    public Servo fClaw;
    public Servo bClaw;

    double armRotate;
    double armExtend;

    public void init()
    {
        //Drive motors -- make sure config file matches the name of the motor
        //You'll probably need to adjust the motor directions. Two motors will need to be reverse.
        //  Which two depends on the robot.
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backLeft.setDirection(DcMotor.Direction.FORWARD);

        techArm = hardwareMap.get(DcMotor.class, "techArm");
        techArm.setDirection(DcMotor.Direction.FORWARD);
        extend = hardwareMap.get(DcMotor.class, "extend");
        extend.setDirection(DcMotor.Direction.FORWARD);

        elbow = hardwareMap.get(Servo.class, "elbow");
        fClaw = hardwareMap.get(Servo.class, "fClaw");
        bClaw = hardwareMap.get(Servo.class, "bClaw");
    }

    public void start()
    {
    }

    public void loop()
    {

        armRotate = gamepad2.left_stick_y;
        armExtend = gamepad2.right_stick_y;

        //this method is what drives the mecanum chassis
        joystickDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y);

        techArm.setPower(armRotate*1);
        extend.setPower(armExtend*.5);

        if(gamepad2.a)
        {
            fClaw.setPosition(.9); bClaw.setPosition(.9);
        }

        if(gamepad2.y)
        {
            fClaw.setPosition(0.0);
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
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        techArm.setPower(0.0);
        extend.setPower(0.0);
    }

    //using joystick values, calculates the proper wheel powers for a mecanum chassis
    public void joystickDrive(double leftStickX, double leftStickY, double rightStickX, double rightStickY)
    {
        double forward = leftStickY;
        double right = -leftStickX;
        double clockwise = -rightStickX;

        double frontLeft2 = (forward + clockwise + right);
        double frontRight2 = (forward - clockwise - right);
        double backLeft2 = (forward + clockwise - right);
        double backRight2 = (forward - clockwise + right);

        frontRight.setPower(frontRight2);
        frontLeft.setPower(frontLeft2);
        backRight.setPower(backRight2);
        backLeft.setPower(backLeft2);
    }
}
