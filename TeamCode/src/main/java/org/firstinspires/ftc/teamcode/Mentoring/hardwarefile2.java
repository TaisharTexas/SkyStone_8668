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
public class hardwarefile2 {
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
    Servo Rotation = null;
    DcMotor LinearSlide = null;


    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public hardwarefile2() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, Telemetry telem) {
        // Save reference to Hardware map

        hwMap = ahwMap;
        telemetry = telem;

        // Define and Initialize Motors
        try {

            frontLeft = hwMap.get(DcMotor.class, "front_left");
            frontLeft.setDirection(DcMotor.Direction.REVERSE);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception p_exception) {
            telem.addData("frontleft is not found in config file", "");
        }
        try {
            frontRight = hwMap.get(DcMotor.class, "front_right");
            frontRight.setDirection(DcMotor.Direction.FORWARD);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        } catch (Exception p_exception) {
            telem.addData("frontright is not found in config file", "");
        }
        try {
            backLeft = hwMap.get(DcMotor.class, "back_left");
            backLeft.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        } catch (Exception p_exception) {
            telem.addData("backLeft is not found in config file", "");
        }
        try {
            backRight = hwMap.get(DcMotor.class, "back_right");
            backRight.setDirection(DcMotor.Direction.FORWARD);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        } catch (Exception p_exception) {
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

    //I changed the if statment logic in setDriveMotorPower and setDriveMotorPowerSideways from
    // using a == sign to using the .equalsIgnoreCase() method.

    // I did this because in java, you're not supposed to use the == sign to compare strings,
    // rather you're supposed to use the .equals() method. .equalsIgnoreCase is the same as a normal
    // .equals() but with the added feature of not worrying about the capitalization of the strings
    // being compared.
    // That way you don't have to worry about remembering whether or not "forward"
    // or "left" starts with a capitol letter or not (I noticed "Left" and "Right" are capitalized
    // while "forward" and "backward" are not.
    public void delay(double delayTime) {
        double startTime = System.currentTimeMillis(); //500
        while ((System.currentTimeMillis() - startTime) < delayTime) {
            telemetry.addData("Robot is waiting", System.currentTimeMillis() - startTime);
            telemetry.update();
        }
    }
    public void setDriveMotorPower(String Direction, double power) {
        backRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        if (Direction.equalsIgnoreCase("forward")) {
            frontRight.setPower(power);
            frontLeft.setPower(power);
            backRight.setPower(power);
            backLeft.setPower(power);

        } else if (Direction.equalsIgnoreCase("backward")) {
            frontRight.setPower(-power);
            frontLeft.setPower(-power);
            backRight.setPower(-power);
            backLeft.setPower(-power);

        }


    }

    public void setDriveMotorPowerSideways(String Direction, double power) {
        backRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        if (Direction.equalsIgnoreCase("Left")) {

            frontRight.setPower(-power);
            frontLeft.setPower(power);
            backRight.setPower(power);
            backLeft.setPower(-power);
        } else if (Direction.equalsIgnoreCase("Right")) {

            backRight.setPower(power);
            backLeft.setPower(-power);
            frontLeft.setPower(power);
            frontRight.setPower(-power);
        }

    }
    public void JoystickDrive(double leftStickX, double leftStickY, double rightStickX, double rightStickY){
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        double forward = leftStickY;
        double right = -leftStickX;
        double clockwise = rightStickX;

        double frontLeft2 = (forward + clockwise + right);
        double frontRight2 = (forward - clockwise - right);
        double backLeft2 = (forward + clockwise - right);
        double backRight2 = (forward - clockwise + right);

        frontRight.setPower(frontRight2);
        frontLeft.setPower(frontLeft2);
        backRight.setPower(backRight2);
        backLeft.setPower(backLeft2);

//        telemetry.addData("leftFront: ", frontLeft2);
//        telemetry.addData("rightFront: ", frontRight2);
//        telemetry.addData("rightBack: ", backRight2);
//        telemetry.addData("leftBack: ", backLeft2);
    }

    public void TurnS(boolean ValueT, double powerL, double powerR) {

        if (ValueT) {
            //initialPositionR = backRight.getCurrentPosition();
            telemetry.addData("Turning Right", "Good Job");
            telemetry.update();
            backRight.setDirection(DcMotor.Direction.REVERSE);
            frontLeft.setDirection(DcMotor.Direction.FORWARD);
            frontRight.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setDirection(DcMotor.Direction.FORWARD);

            telemetry.update();
            backLeft.setPower(powerL);
            backRight.setPower(powerR);
            frontLeft.setPower(powerL);
            frontRight.setPower(powerR);
        }else {
            telemetry.addData("Turning Left", "Good Job");
            telemetry.update();
            backLeft.setDirection(DcMotor.Direction.REVERSE);
            frontRight.setDirection(DcMotor.Direction.FORWARD);
            frontLeft.setDirection(DcMotor.Direction.REVERSE);
            backRight.setDirection(DcMotor.Direction.FORWARD);

            backLeft.setPower(powerL);
            backRight.setPower(powerR);
            frontLeft.setPower(powerL);
            frontRight.setPower(powerR);
        }
    }
    public void stopDriveMotors()
    {
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }
}


