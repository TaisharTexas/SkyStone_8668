package org.firstinspires.ftc.teamcode.Mentoring;
//package org.firstinspires.ftc.teamcode;


import android.sax.TextElementListener;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

        switch (state) {

            //I did not touch case 0, instead I added cases 1 and 2, demonstrating my suggested changes.
            case 0:

                backrightInches = 12;
                newBackRightTarget = robot.backRight.getCurrentPosition() + (int)(backrightInches * COUNTS_PER_INCH);
                robot.setDriveMotorPower("forward", 0.5);
                telemetry.addData("case: ", state);
                if(robot.backRight.getCurrentPosition()>= newBackRightTarget )
                {
                    robot.stopDriveMotors();
                    runtime.reset();
//                    state++;
                    //I specified state=2; so that case 1 would be skipped since it won't work.
                    // If you want to try it out just to see what happens, change it back to state++;
                    state = 2;
                    telemetry.addData("case: ", state);
                }
                break;


                //case 1 isn't gong to work properly. The logic doesn't support a target smaller than your current position.
                // see case 3 for updated logic
            case 1:
                targetDistanceInches = 12; //the desired number of inches to move
                //calculating how many encoder ticks will be required to move desired number of inches.
                newBackRightTarget = robot.backRight.getCurrentPosition() + (int)(targetDistanceInches * COUNTS_PER_INCH);
                //setting all drive motors to .5 power
                robot.setDriveMotorPower("backward", 0.5);
                //once the current position of the robot reaches the desired target, stop the motors
                if(robot.backRight.getCurrentPosition() >= newBackRightTarget)
                {
                    //set drive motors to 0 power
                    robot.stopDriveMotors();
                    //reset runtime timer
                    runtime.reset();
                    //increment state value to the next value
                    state++;
                }
                break;

            case 2:
                targetDistanceInches = 12;
                newBackRightTarget = robot.backRight.getCurrentPosition() + (int)(targetDistanceInches*COUNTS_PER_INCH);
                robot.setDriveMotorPower("forward", 1);
                if(robot.backRight.getCurrentPosition() >= newBackRightTarget)
                {
                    robot.stopDriveMotors();
                    runtime.reset();
                    encoder = robot.backLeft.getCurrentPosition();
                    state++;
                }
                break;


                //Case 3 uses logic for driving that allows for the target being less than the current position.
                // This requires that you capture the robot's position at the end of every move.

                //The logic goes like this: take the robot's current position and subtract the
                // position of the robot captured at the end of last move (which is also the
                // beginning of this move), and take the absolute value of the result.
                // If the result is greater than the absolute value of the target, then the robot
                // has reached the desired position and can stop.

                //Basically you're using a delta of your position to deal with any negative
                // position/target values.

                // -- This is the logic that we used all the up until worlds two years ago. After that we
                // moved away from a state machine, although the logic is still present in our current code.
                // The logic in case 3 works for forward and backward moves as well as strafing moves.
                // Y'all are welcome to do what you will with this.

            case 3:
                //desired number of inches to travel
                targetDistanceInches = 12;
                //convert desired number of inches to encoder ticks
                newBackRightTarget = (int)(targetDistanceInches * COUNTS_PER_INCH);
                //set motor durectio and power
                robot.setDriveMotorPower("backward", .5);
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
                    encoder = robot.backLeft.getCurrentPosition();
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
