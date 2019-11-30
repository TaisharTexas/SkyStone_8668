package org.firstinspires.ftc.teamcode.Mentoring;
//package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 *
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 *
 * @author Robo Sapiens FTC
 */
public class hardwarefile2
{
    /* Public OpMode members. */
    DcMotor frontLeft = null;
    DcMotor frontRight = null;
    DcMotor backLeft = null;
    DcMotor backRight = null;
    Telemetry telemetry;


    //    DcMotor gripMotor = null;
//    Servo leftGrip = null;
//    Servo rightGrip = null;
//        ColorSensor colorSensor = null;
    Servo autoServo = null;
    Servo rightFoundation = null;
    Servo leftFoundation = null;
    DcMotor LeftCollector = null;
    DcMotor RightCollector = null;
    Servo Grabber = null;
    Servo Rotation = null;DcMotor LinearSlide = null;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public hardwarefile2(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, Telemetry telem)
    {
        // Save reference to Hardware map

        hwMap = ahwMap;
        telemetry = telem;

        // Define and Initialize Motors
        try {

            frontLeft = hwMap.get(DcMotor.class, "front_left");
            frontLeft.setDirection(DcMotor.Direction.REVERSE);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        catch (Exception p_exception) {
            telem.addData("frontleft is not found in config file", "");
        }
        try {
            frontRight = hwMap.get(DcMotor.class, "front_right");
            frontRight.setDirection(DcMotor.Direction.FORWARD);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
        catch (Exception p_exception) {
            telem.addData("frontright is not found in config file", "");
        }
        try {
            backLeft = hwMap.get(DcMotor.class, "back_left");
            backLeft.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
        catch (Exception p_exception) {
            telem.addData("backLeft is not found in config file", "");
        }
        try {
            backRight = hwMap.get(DcMotor.class, "back_right");
            backRight.setDirection(DcMotor.Direction.FORWARD);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
        catch (Exception p_exception) {
            telem.addData("backright is not found in config file", "");
        }


        Rotation = hwMap.get(Servo.class, "rotationservo");
        Grabber = hwMap.get(Servo.class, "grabbername");
        leftFoundation = hwMap.get(Servo.class, "left_foundation");
        rightFoundation = hwMap.get(Servo.class, "right_foundation");
        // Set to REVERSE if using AndyMark motors
        // Set to REVERSE if using AndyMark motors
        // Set to FORWARD if using AndyMark motors

        rightFoundation.setPosition(1);
        leftFoundation.setPosition(0);

    }
}

