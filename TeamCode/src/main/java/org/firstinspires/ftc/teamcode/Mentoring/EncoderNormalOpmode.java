package org.firstinspires.ftc.teamcode.Mentoring;
//package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * The autonomous OpMode. Contains a state machine that iterates through each autonomous step.
 * Sources all the hardware from hardwarefile2
 *
 * @author Robot Sapiens FTC
 * @see OpMode
 * */

import org.firstinspires.ftc.robotcore.external.Telemetry;

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

    static final double     COUNTS_PER_MOTOR_REV    = 95.9 ;    // eg: TETRIX Motor Encoder
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
        switch(state) {

            //I did not touch case 0, instead I added cases 1 and 2, demonstrating my suggested changes.
            case 0:
                backrightInches = 12;
                newBackRightTarget = robot.backRight.getCurrentPosition() + (int)(backrightInches * COUNTS_PER_INCH);
                robot.frontRight.setPower(1);
                robot.frontLeft.setPower(1);
                robot.backRight.setPower(1);
                robot.backLeft.setPower(1);
                if(robot.backRight.getCurrentPosition()>= newBackRightTarget )
                {
                    robot.frontRight.setPower(0);
                    robot.frontLeft.setPower(0);
                    robot.backRight.setPower(0);
                    robot.backLeft.setPower(0);
                    runtime.reset();
                    state++;
                }
                break;

            case 1:
                targetDistanceInches = 9; //the desired number of inches to move
                //calculating how many encoder ticks will be required to move desired number of inches.
                newBackRightTarget = robot.backRight.getCurrentPosition() + (int)(targetDistanceInches * COUNTS_PER_INCH);
                //setting all drive motors to .5 power
                setDriveMotorPower(.5);
                //once the current position of the robot reaches the desired target, stop the motors
                if(robot.backRight.getCurrentPosition() >= newBackRightTarget)
                {
                    //set drive motors to 0 power
                    setDriveMotorPower(0.0);
                    //reset runtime timer
                    runtime.reset();
                    //increment state value to the next value
                    state++;
                }
                break;

            case 2:
                targetDistanceInches = 20;
                newBackRightTarget = robot.backRight.getCurrentPosition() + (int)(targetDistanceInches*COUNTS_PER_INCH);
                setDriveMotorPower(1);
                if(robot.backRight.getCurrentPosition() >= newBackRightTarget)
                {
                    setDriveMotorPower(0.0);
                    runtime.reset();
                    state++;
                }
                break;

              default:
                    break;



            /* NOTE:
            *
            * What you have here only works for driving forward. You don't have anything that can drive
            * backwards or turn. Yes, technically you can command the motors to drive backwards
            * using a negative power, but the if- statement that you have checking to see how far the
            * robot has gone and when to stop it can't handle the target being smaller than the
            * current position. You'll need a more robust way of comparing your current location with
            * your target location. I suggest building a new drive method in hardwarefile2 that can
            * handle both forwards and backwards.
            *
            * Also, you'll need to make a new method (I'd put it in hardwarefile2) that can let the
            * robot turn.
            *
            *  */



        }

    }

    public void setDriveMotorPower(double power)
    {
        robot.frontRight.setPower(power);
        robot.frontLeft.setPower(power);
        robot.backRight.setPower(power);
        robot.backLeft.setPower(power);
    }

}
