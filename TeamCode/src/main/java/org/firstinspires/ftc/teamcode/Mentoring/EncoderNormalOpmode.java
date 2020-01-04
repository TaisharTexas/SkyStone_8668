package org.firstinspires.ftc.teamcode.Mentoring;
//package org.firstinspires.ftc.teamcode;


import android.sax.TextElementListener;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * The autonomous OpMode. Contains a state machine that iterates through each autonomous step.
 * Sources all the hardware from hardwarefile2
 *
 * @author Robot Sapiens FTC
 * @see OpMode
 * */
//@Disabled
@Autonomous(name="Normal OpMode Testing", group="Pushbot")
public class EncoderNormalOpmode extends OpMode
{
    hardwarefile2      robot   = new hardwarefile2();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();


    double backrightInches; //the number of inches you want to move

    double targetDistanceInches; //I suggest changing ^the above variable to this variable as this
                                 // variable is more descriptive of what the variable is actually
                                 // being used for.
    int newBackRightTarget;

    double encoder;

    static final double     COUNTS_PER_MOTOR_REV    = 383.6 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.94 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.5;
    static final double     TURN_SPEED = 0.5;
    int state = 0;


    /*
    *
    * THE BIG CHANGE:
    *
    * The error I found was that we were declaring hardware inside this class that was overriding
    * the hardware we had declared inside the hardwarefile2 class. Since you already declare and
    * initialize all your hardware (motors, servos, etc) in hardwarefile2, you don't need to declare
    * new hardware components inside this class, but rather use the ones in hardwarefile2. The problem
    * was that we brought in the drive motors from hardwarefile2 and then declared motors with the
    * same names as the motors we had just declared in hardwarefile2, except the new motors weren't
    * initialized, hence the null pointer error.
    *
    * */



    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);


    }

    @Override
    public void loop() {
    telemetry.addData("robot should start with the switch states", "");
    telemetry.addData("state:",state);
    telemetry.addData("backLeft: ", robot.backLeft.getPower());
    telemetry.addData("backRight: ", robot.backRight.getPower());
    telemetry.addData("frontLeft: ", robot.frontLeft.getPower());
    telemetry.addData("frontRight: ", robot.frontRight.getPower());
    telemetry.update();
        switch (state) {

            case 0:
                //desired number of inc00s to travel
                targetDistanceInches = 15;
                //convert desired number of inches to encoder ticks
                newBackRightTarget = (int)(targetDistanceInches * COUNTS_PER_INCH);
                //set motor durectio and power
                robot.setDriveMotorPower("backward",0.5);
                telemetry.addData("backLeft: ", robot.backLeft.getPower());
                telemetry.addData("backRight: ", robot.backRight.getPower());
                telemetry.addData("frontLeft: ", robot.frontLeft.getPower());
                telemetry.addData("frontRight: ", robot.frontRight.getPower());
                telemetry.addData("case: ", state);
                //if the absolute value of the current position minus the position from the end of the last move is
                // greater than the absolute value of the target, do the things inside the if- statement
                if(Math.abs(robot.backRight.getCurrentPosition()-encoder) >= Math.abs(newBackRightTarget))
                {
                    //stop the motors
                    robot.stopDriveMotors();
                    //reset the timer
                    runtime.reset();
                    //record the robot's position for use in the next move
                    encoder = robot.backRight.getCurrentPosition();
                    //increment the state value
                    state++;
                    telemetry.addData("case: ", state);
                }
                break;

            case 2:
                if(getRuntime()>3)
                {
                    state++;
                    resetStartTime();
                }
                break;
            case 3:// go slower and right
                //desired number of inc00s to travel
                targetDistanceInches = 10;
                //convert desired number of inches to encoder ticks
                newBackRightTarget = (int)(targetDistanceInches * COUNTS_PER_INCH);
                //set motor durectio and power
                robot.setDriveMotorPowerSideways("right",0.5);
                telemetry.addData("backLeft: ", robot.backLeft.getPower());
                telemetry.addData("backRight: ", robot.backRight.getPower());
                telemetry.addData("frontLeft: ", robot.frontLeft.getPower());
                telemetry.addData("frontRight: ", robot.frontRight.getPower());
                telemetry.addData("case: ", state);
                //if the absolute value of the current position minus the position from the end of the last move is
                // greater than the absolute value of the target, do the things inside the if- statement
                if(Math.abs(robot.backRight.getCurrentPosition()-encoder) > Math.abs(newBackRightTarget))
                {
                    //stop the motors
                    robot.stopDriveMotors();
                    //reset the timer
                    runtime.reset();
                    //record the robot's position for use in the next move
                    encoder = robot.backRight.getCurrentPosition();
                    //increment the state value
                    state++;
                    telemetry.addData("case: ", state);
                }
                break;

            case 4:
                //desired number of inches to travel
                robot.delay(500);
                robot.rightFoundation.setPosition(0.78);
                robot.leftFoundation.setPosition(0.33);
                robot.delay(500);
                targetDistanceInches = 18;
                //convert desired number of inches to encoder ticks
                newBackRightTarget = (int)(targetDistanceInches * COUNTS_PER_INCH);
                //set motor durectio and power
                robot.JoystickDrive(0,-1,0,0);
                telemetry.addData("case: ", state);
                //if the absolute value of the current position minus the position from the end of the last move is
                // greater than the absolute value of the target, do the things inside the if- statement
                if(Math.abs(robot.backRight.getCurrentPosition()-encoder) > Math.abs(newBackRightTarget))
                {
                    //stop the motors
                    robot.stopDriveMotors();
                    //reset the timer
                    runtime.reset();
                    //record the robot's position for use in the next move
                    encoder = robot.backRight.getCurrentPosition();
                    //increment the state value
                    state++;
                    telemetry.addData("case: ", state);
                }
                break;
            case 5:
                robot.delay(500);
                //desired number of inches to travel
                robot.rightFoundation.setPosition(1);
                robot.leftFoundation.setPosition(0);
                targetDistanceInches = 24;
                //convert desired number of inches to encoder ticks
                newBackRightTarget = (int)(targetDistanceInches * COUNTS_PER_INCH);
                //set motor durectio and power
                robot.JoystickDrive(0.5,0,0,0);
                telemetry.addData("case: ", state);
                //if the absolute value of the current position minus the position from the end of the last move is
                // greater than the absolute value of the target, do the things inside the if- statement
                if(Math.abs(robot.backRight.getCurrentPosition()-encoder) > Math.abs(newBackRightTarget))
                {
                    //stop the motors
                    robot.stopDriveMotors();
                    //reset the timer
                    runtime.reset();
                    //record the robot's position for use in the next move
                    encoder = robot.backRight.getCurrentPosition();
                    //increment the state value
                    state++;
                    telemetry.addData("case: ", state);
                }
                break;
            default:
                    break;



        }

    }


}
