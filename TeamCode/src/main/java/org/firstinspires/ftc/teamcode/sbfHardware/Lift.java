package org.firstinspires.ftc.teamcode.sbfHardware;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.concurrent.TimeUnit;

/**
 * Contains the hardware and usage methods for the lift mechanism.
 *
 * @author Andrew, 8668 Should Be Fine!
 */
public class Lift
{
    /** A telemetry object passed down from the opmode. */
    private Telemetry telemetry;
    /** A telemetry object passed down from the opmode. */
    private HardwareMap hardwareMap;

    /** The left touch sensor. Used to keep the lift from driving down into the chassis. */
    public RevTouchSensor touchL = null;
    /** The right touch sensor. Used to keep the lift from driving down into the chassis. */
    public RevTouchSensor touchR = null;

    /** Declaring the left lift motor as an expanded rev hub motor. */
    public ExpansionHubMotor leftVertical = null;
    /** Declaring the right lift motor as an expanded rev hub motor. */
    public ExpansionHubMotor rightVertical = null;
    /** Declaring the vertical encoder */
    public ExpansionHubMotor vLiftEncoder = null;
    /** Declaring the horizontal encoder */
    public ExpansionHubMotor hLiftEncoder = null;

    /** The servo that drives the horizontal slides. */
    public Servo horizontal = null;
    /** The servo that drives the stone gripper. */
    public Servo claw = null;
    /** The servo that drives the gripper wrist. */
    public Servo wrist = null;
    /** The servo that releases the capstone. */
    public Servo capStone = null;
    public CRServo vexHoriz = null;

    /** Stores the current value of the left lift motor's encoder. */
    public int vEncoder = 0;
    public int hEncoder = 0;

    /** The IN position for the horizontal slides. */
    private final double HOME = .29;
    /** The OUT position for the horizontal slides. */
    private final double EXTEND = .46;
    /** Stores the current state for the goHome() state machine. */
    int state = 0;
    /** Stores the current state for the autoExtend() state machine. */
    int stateTwo = 0;
    // internal time tracking
    /** Sets the internal timer to zero (in nanoseconds). */
    private long startTime = 0; // in nanoseconds

    /** The number of encoder ticks per rotation of the lift motor output shaft. */
    private final double ENCODER_TICKS_PER_MTR_ROTATION = 3360; //encoder ticks
    /** The number of inches of string moved (around the spools) per rotation of the lift motor output shaft. */
    private final double ROTATIONAL_IN_PER_MTR_ROTATION = 9.4; //inches rotation
    /** The number of inches of vertical movement (the lift slides) per inch of string moved around the spools.  */
    private final double ROTATIONAL_IN_PER_VERTICAL_IN = .43; //inches vertical
    /** The number of encoder ticks per inch of vertical movement in the lift slides. */
    private final double ENCODER_TICKS_PER_IN_VERTICAL = (ENCODER_TICKS_PER_MTR_ROTATION*ROTATIONAL_IN_PER_VERTICAL_IN)/(ROTATIONAL_IN_PER_MTR_ROTATION);
    /** Marks whether or not the lift mechanism is moving. */
    private boolean moving;
    /** Stores the initial position of the lift to be used in calculating the delta position of the lift. */
    private double initialPosition;

    /**
     * Runs once when INIT is pressed on the driver station. Sets up all the hardware used by the class.
     * @param telem  A telemetry object passed down from the opmode.
     * @param hwmap  A hardware map object passed down from the opmode.
     */
    public void init(Telemetry telem, HardwareMap hwmap)
    {
        telemetry = telem;
        hardwareMap = hwmap;
        moving = false;

//        try
//        {
//            leftVertical = (ExpansionHubMotor) hardwareMap.get(DcMotorEx .class, "leftV");
//            leftVertical.setDirection(DcMotorEx.Direction.REVERSE);
//            leftVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            leftVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            leftVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        }
//        catch (Exception p_execption)
//        {
//            leftVertical = null;
//            telemetry.addData("left vertical not found in config file", "");
//        }
//
//        try
//        {
//            rightVertical = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, "rightV");
//            rightVertical.setDirection(DcMotorEx.Direction.FORWARD);
//            rightVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            rightVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            rightVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
//        catch (Exception p_execption)
//        {
//            rightVertical = null;
//            telemetry.addData("right vertical not found in config file", "");
//        }


/////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //ODOMETERS -- MUST INITIALIZE BEFORE THE VERTICAL MOTORS
        try
        {
            vLiftEncoder = (ExpansionHubMotor)hardwareMap.get(DcMotorEx.class, "vEncoder");
            vLiftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            vLiftEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        catch(Exception p_exception)
        {
            vLiftEncoder = null;
            telemetry.addData("vertical lift encoder ", "not found in config file");
        }
        try
        {
            hLiftEncoder = (ExpansionHubMotor)hardwareMap.get(DcMotorEx.class, "hEncoder");
            hLiftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hLiftEncoder.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        catch(Exception p_exception)
        {
            hLiftEncoder = null;
            telemetry.addData("horizontal lift encoder ", "not found in config file");
        }
        //NORMAL LIFT MOTORS -- MUST INITIALIZE AFTER THE ODOMETERS
        try
        {
            leftVertical = (ExpansionHubMotor) hardwareMap.get(DcMotorEx .class, "vEncoder");
            leftVertical.setDirection(DcMotorEx.Direction.REVERSE);
            leftVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftVertical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }
        catch (Exception p_execption)
        {
            leftVertical = null;
            telemetry.addData("left vertical not found in config file", "");
        }

        try
        {
            rightVertical = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, "hEncoder");
            rightVertical.setDirection(DcMotorEx.Direction.FORWARD);
            rightVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightVertical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        catch (Exception p_execption)
        {
            rightVertical = null;
            telemetry.addData("right vertical not found in config file", "");
        }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        try
        {
            touchL = hardwareMap.get(RevTouchSensor.class, "touchL");

        }
        catch (Exception p_execption)
        {
            touchL = null;
            telemetry.addData("left touch not found in config file", "");
        }
        try
        {
            touchR = hardwareMap.get(RevTouchSensor.class, "touchR");
        }
        catch (Exception p_execption)
        {
            touchR = null;
            telemetry.addData("right touch not found in config file", "");
        }
//        try
//        {
//            horizontal = hardwareMap.get(Servo.class, "horizontal");
//            horizontal.setDirection(Servo.Direction.REVERSE);
////            horizontal.setPosition(.29);
//        }
//        catch (Exception p_exeception)
//        {
//            telemetry.addData("horizontal not found in config file", 0);
//            horizontal = null;
//        }
//        try
//        {
//            capStone = hardwareMap.get(Servo.class, "cap");
//            capStone.setDirection(Servo.Direction.REVERSE);
//            capStone.setPosition(.6);
//        }
//        catch (Exception p_exeception)
//        {
//            telemetry.addData("capStone Servo not found in config file", 0);
//            capStone = null;
//        }
//        try
//        {
//            claw = hardwareMap.get(Servo.class, "claw");
//            claw.setDirection(Servo.Direction.FORWARD);
//        }
//        catch (Exception p_exeception)
//        {
//            telemetry.addData("claw not found in config file", 0);
//            claw = null;
//        }
//        try
//        {
//            wrist = hardwareMap.get(Servo.class, "wrist");
//            wrist.setDirection(Servo.Direction.FORWARD);
//        }
//        catch (Exception p_exeception)
//        {
//            telemetry.addData("wrist not found in config file", 0);
//            wrist = null;
//        }
//        try
//        {
//            vexHoriz = hardwareMap.get(CRServo.class, "vexH");
//            vexHoriz.setDirection(CRServo.Direction.REVERSE);
//        }
//        catch (Exception p_exeception)
//        {
//            telemetry.addData("vexHorizontal Servo not found in config file", 0);
//            vexHoriz = null;
//        }

    }

    /**
     * Drives the lift to a set position (inches).
     *
     * @param power  How fast the lift will drive.
     * @param positionInches  Where the lift will be driven to in inches.
     * @param time  The max time this move can take. A time-out feature: if the move stalls for some
     *              reason, the timer will catch it.
     * @return  A boolean that tells us whether or not the lift is moving.
     */
    public boolean vLiftDrive(double power, double positionInches, double time)
    {
        double driveDistance = ENCODER_TICKS_PER_IN_VERTICAL * positionInches;
        telemetry.addData("drive distance: ", driveDistance);
        telemetry.addData("moving: ", moving);
        telemetry.addData("encoder: ", vEncoder);
        double gain = 1;
        double liftEncoderPos = vEncoder;

        if (!moving)
        {
            initialPosition = liftEncoderPos;
            resetLiftStartTime();
            moving = true;
        }

        verticalDrive(-power);
//        leftVertical.setPower(power);
//        rightVertical.setPower(power);

        if (((Math.abs(liftEncoderPos - initialPosition)) >= driveDistance) || (getLiftRuntime() > time))
        {
            stopLift();
            moving = false;

        }

        return !moving;
    }

    /** Drives the lift at a set power (sign of power determines direction).
     * Only runs if the motors are properly initialized and has several conditions that can override the user command:
     * 1) If the lift is fully extended and the user sends a command that woudl overdrive the lift, block that command.
     * 2) If either touch sensor is pressed, automatically reverse the direction of the lift until
     * the sensor is released (prevents the lift from being driven down into the chassis).
     * 3) When the lift gets close to the down position, slow the lift down by fifty percent
     * 4) If none of the above conditions are met, operate the lift as commanded by the user.
     * @param power  The power at which the lift is driven.
     * */
    public void verticalDrive(double power)
    {
        if(isVLiftValid())
        {
            telemetry.addData("velocity: ", leftVertical.getVelocity());

            if ( vEncoder > 9000 && power < 0 ) {
                leftVertical.setPower(0.0);
                rightVertical.setPower(0.0);
            }
            else if(touchR.isPressed() || touchL.isPressed())
            {
                leftVertical.setPower(.35);
                rightVertical.setPower(.35);
            }
            else if ( vEncoder < 700  && power > 0)
            {
                leftVertical.setPower(Range.clip(-power,-.5,.5));
                rightVertical.setPower(Range.clip(-power,-.5,.5));
            }

            else if(power > .01 || power < -.01)
            {
                leftVertical.setPower(-power);
                rightVertical.setPower(-power);
            }

            else
            {
                leftVertical.setPower(0.0);
                rightVertical.setPower(0.0);
            }
        }
        else
        {
            telemetry.addData("something in the lift"," equals null");
        }


    }

    /** Drives the horizontal slides to a specified position.
     * @param power  The position the slides are driven to.
     * */
    public void horizontalDrive(double power)
    {
        if(horizontal != null)
        {
            telemetry.addData("horizontal pwr: ", vexHoriz.getPower());
            telemetry.addData("horizontal pos: ", hEncoder);
            vexHoriz.setPower(power);
        }
    }

    /**
     * Drives the horizontal lift to a set position (inches).
     *
     * @param power  How fast the lift will drive.
     * @param positionInches  Where the lift will be driven to in inches.
     * @param time  The max time this move can take. A time-out feature: if the move stalls for some
     *              reason, the timer will catch it.
     * @return  A boolean that tells us whether or not the lift is moving.
     */
    public boolean hLiftDrive(double power, double positionInches, double time)
    {
        double driveDistance = positionInches;
        double gain = 1;
        double liftEncoderPos = hEncoder;

        if (!moving)
        {
            initialPosition = liftEncoderPos;
            resetLiftStartTime();
            moving = true;
        }

        horizontalDrive(-power);

        if (((Math.abs(liftEncoderPos - initialPosition)) >= driveDistance) || (getLiftRuntime() > time))
        {
            stopLift();
            moving = false;
        }
        return !moving;
    }

    /**
     * Closes the stone gripper.
     */
    public void grabClaw()
    {
        if(claw != null)
        {
            claw.setPosition(.8);
        }
    }

    /**
     * Opens the stone gripper.
     */
    public void releaseClaw()
    {
        if(claw != null)
        {
            claw.setPosition(.6);
        }
    }

    /**
     * Turns the gripper to ninety degrees -- for placing a stone.
     */
    public void wristDeploy()
    {
        if(wrist != null)
        {
//            wrist.setPosition(.9);
            wrist.setPosition(.45);
        }

    }

    /**
     * Turns the wrist to zero degrees -- for grabbing another stone.
     */
    public void wristRetract()
    {
        if(wrist != null)
        {
            wrist.setPosition(.05);
        }
    }

    /**
     * Turns the wrist to a specified position.
     * @param position  The position the wrist is to be turned to.
     */
    public void wristDrive( double position )
    {
        if(horizontal != null)
        {
            wrist.setPosition(position);
        }
    }

    /**
     * Get the number of seconds this op mode has been running
     * <p>
     * This method has sub millisecond accuracy.
     * @return number of seconds this op mode has been running
     */
    private double getLiftRuntime() {
        final double NANOSECONDS_PER_SECOND = TimeUnit.SECONDS.toNanos(1);
        return (System.nanoTime() - startTime) / NANOSECONDS_PER_SECOND;
    }

    /**
     * Reset the start time to zero.
     */
    private void resetLiftStartTime() {
        startTime = System.nanoTime();
    }

    /**
     * Stops the lift motors.
     */
    public void stopLift()
    {
        if(isVLiftValid())
        {
            rightVertical.setPower(0.0);
            leftVertical.setPower(0.0);
        }
        if(vexHoriz!=null)
        {
            vexHoriz.setPower(0.0);
        }
    }

    /**
     * Checks to make sure both lift motors were initialized properly.
     * @return true if the motors are fine, false if the motors are null.
     */
    private boolean isVLiftValid()
    {
        if(leftVertical!=null && rightVertical!=null)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    public void releaseCap()
    {
        if(capStone!=null)
        {
            capStone.setPosition(.2);
        }
    }


}
