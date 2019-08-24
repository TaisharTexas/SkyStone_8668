package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name="pursuit run", group="pure")

public class MecanumPursuit extends OpMode
{
    Vehicle robot = new Vehicle((float)0.0, (float)0.0);

//    private DcMotor testEncoder;
    private DcMotor RF = null;
    private DcMotor RR = null;
    private DcMotor LF = null;
    private DcMotor LR = null;

    private DcMotor xEncoder = null;
    private DcMotor yEncoder = null;

    final double tickPerRotation = 2400;
    final double inchesPerRotation = 3 * Math.PI;

    Telemetry telem;
    HardwareMap hwMap;
    PVector target1 = new PVector(0,-30);


    public void init()
    {

        RF  = hardwareMap.get(DcMotor.class, "rf");
        RF.setDirection(DcMotor.Direction.FORWARD);
        RR  = hardwareMap.get(DcMotor.class, "rr");
        RR.setDirection(DcMotor.Direction.FORWARD);
        LF  = hardwareMap.get(DcMotor.class, "lf");
        LF.setDirection(DcMotor.Direction.FORWARD);
        LR  = hardwareMap.get(DcMotor.class, "lr");
        LR.setDirection(DcMotor.Direction.FORWARD);

        xEncoder  = hardwareMap.get(DcMotor.class, "xEncoder");
        xEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xEncoder.setDirection((DcMotor.Direction.FORWARD));

        yEncoder  = hardwareMap.get(DcMotor.class, "yEncoder");
        yEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yEncoder.setDirection((DcMotor.Direction.FORWARD));


    }

    public void loop()
    {
        /* Chassis Control */
        /** The x-axis of the left joystick on the gamepad. Used for chassis control*/
        double lStickX = -gamepad1.left_stick_x;
        /** The x-axis of the right joystick on the gamepad. Used for chassis control*/
        double rStickX = -gamepad1.right_stick_x;
        /** The y-axis of the left joystick on the gamepad. Used for chassis control*/
        double lStickY = gamepad1.left_stick_y;
        /** The y-axis of the right joystick on the gamepad. Used for chassis control*/
        double rStickY = gamepad1.right_stick_y;

//        RF.setPower(-gamepad1.right_stick_y);
//        RR.setPower(-gamepad1.right_stick_y);
//        LF.setPower(gamepad1.left_stick_y);
//        LR.setPower(gamepad1.left_stick_y);
//        joystickDrive(lStickX, lStickY, rStickX, rStickY, .5);
        robot.location.set(xEncoder.getCurrentPosition(), yEncoder.getCurrentPosition());

        updateMotors(target1);

        getInchesMoved();

    }

    public void getInchesMoved()
    {
        double inchesX = ((xEncoder.getCurrentPosition() / tickPerRotation) * inchesPerRotation);
        double inchesY = ((yEncoder.getCurrentPosition() / tickPerRotation) * inchesPerRotation);

        telemetry.addData("x inches moved: ", inchesX);
        telemetry.addData("y inches moved: ", inchesY);
    }

    public void updateMotors(PVector targetPosition)
    {
        robot.arrive(target1);
        PVector neededVeloctiy = robot.velocty.copy();
        neededVeloctiy.normalize();

        telemetry.addData("needed velocty: ", neededVeloctiy);

        double x = neededVeloctiy.x;
        double y = neededVeloctiy.y;

        joystickDrive(x, y, 0, 0, .5);
    }

    public void joystickDrive(double leftStickX, double leftStickY, double rightStickX, double rightStickY, double powerLimit)
    {
        /*
            These are the calculations need to make a simple mecaccnum drive.
              - The left joystick controls moving straight forward/backward and straight sideways.
              - The right joystick control turning.
        */
        double rightFront = (-leftStickY+rightStickX+leftStickX);
        double leftFront = (leftStickY+rightStickX+leftStickX);
        double rightRear=  (-leftStickY+rightStickX-leftStickX);
        double leftRear = (leftStickY+rightStickX-leftStickX);


        //Find the largest command value given and assign it to max.
        double max = 0.0;
        if (Math.abs(leftFront) > max)  { max = Math.abs(leftFront); }
        if (Math.abs(rightFront) > max) { max = Math.abs(rightFront); }
        if (Math.abs(leftRear) > max)   { max = Math.abs(leftRear); }
        if (Math.abs(rightRear) > max)  { max = Math.abs(rightRear); }

        //Set the minimum and maximum power allowed for drive moves and compare it to the parameter powerLimit.
        powerLimit = Range.clip(powerLimit, .05, 1);
        //If max still equals zero after checking all four motors, then set the max to 1
        if (max == 0.0)
        {
            max = 1;
        }

        // If max is greater than the power limit, divide all command values by max to ensure that all command
        // values stay below the magnitude of the power limit.
        if (max > powerLimit)
        {
            leftFront = leftFront / max * powerLimit;
            rightFront = rightFront / max * powerLimit;
            leftRear = leftRear / max * powerLimit;
            rightRear = rightRear / max *powerLimit;
        }

        RF.setPower(rightFront);
        RR.setPower(rightRear);
        LF.setPower(leftFront);
        LR.setPower(leftRear);

        //Give the motors the final power values -- sourced from the calculations above.


//////////////////////////////////////////////////////////////////////////
//    ////////if the robot is not moving, instruct the motors to hold their current position.///////
//        if(rightFront == 0 && leftFront == 0)
//        {
//            setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            double rf = rFrontMotor.getCurrentPosition();
//            double lf = lFrontMotor.getCurrentPosition();
//            double rr = rRearMotor.getCurrentPosition();
//            double lr = lRearMotor.getCurrentPosition();
//            if (rFrontMotor != null)
//            {
//                rFrontMotor.setTargetPosition( (int) rf);
//            }
//
//            if (lFrontMotor != null)
//            {
//                lFrontMotor.setTargetPosition( (int) lf);
//            }
//
//            if (rRearMotor != null)
//            {
//                rRearMotor.setTargetPosition( (int) rr);
//            }
//
//            if (lRearMotor != null)
//            {
//                lRearMotor.setTargetPosition( (int) lr);
//            }
//        }
//        else
//        {
//            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
//////////////////////////////////////////////////////////////////////////////////

    }

    public void stop()
    {
//        RF.setPower(0.0);
//        RR.setPower(0.0);
//        LF.setPower(0.0);
//        LR.setPower(0.0);

    }


}
