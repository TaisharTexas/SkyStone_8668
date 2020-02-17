package org.firstinspires.ftc.teamcode.sbfHardware;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

    
    private RevTouchSensor touchL = null;
    private RevTouchSensor touchR = null;

    public ExpansionHubMotor leftVertical = null;
    public ExpansionHubMotor rightVertical = null;

    public Servo horizontal = null;
    public Servo claw = null;
    public Servo wrist = null;

    public int encoder = 0;

    private final double HOME = .29;
    private final double EXTEND = .46;

    double thePosition;
    int state = 0;
    int stateTwo = 0;
    // internal time tracking
    private long startTime = 0; // in nanoseconds

    final double ENCODER_TICKS_PER_MTR_ROTATION = 3360; //encoder ticks
    final double ROTATIONAL_IN_PER_MTR_ROTATION = 9.4; //inches rotation
    final double ROTATIONAL_IN_PER_VERTICAL_IN = .43; //inches vertical
    final double ENCODER_TICKS_PER_IN_VERTICAL = (ENCODER_TICKS_PER_MTR_ROTATION*ROTATIONAL_IN_PER_VERTICAL_IN)/(ROTATIONAL_IN_PER_MTR_ROTATION);
    boolean moving;
    double initialPosition;



    public void init(Telemetry telem, HardwareMap hwmap)
    {
        telemetry = telem;
        hardwareMap = hwmap;
        moving = false;

        try
        {
            leftVertical = (ExpansionHubMotor) hardwareMap.get(DcMotorEx .class, "leftV");
            leftVertical.setDirection(DcMotorEx.Direction.REVERSE);
            leftVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
        catch (Exception p_execption)
        {
            leftVertical = null;
            telemetry.addData("left vertical not found in config file", "");
        }

        try
        {
            rightVertical = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, "rightV");
            rightVertical.setDirection(DcMotorEx.Direction.FORWARD);
            rightVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        catch (Exception p_execption)
        {
            rightVertical = null;
            telemetry.addData("right vertical not found in config file", "");
        }




        touchL = hardwareMap.get(RevTouchSensor.class, "touchL");
        touchR = hardwareMap.get(RevTouchSensor.class, "touchR");

        try
        {
            horizontal = hardwareMap.get(Servo.class, "horizontal");
            horizontal.setDirection(Servo.Direction.REVERSE);
//            horizontal.setPosition(.3);
        }
        catch (Exception p_exeception)
        {
            telemetry.addData("horizontal not found in config file", 0);
            horizontal = null;
        }
        try
        {
            claw = hardwareMap.get(Servo.class, "claw");
            claw.setDirection(Servo.Direction.FORWARD);
        }
        catch (Exception p_exeception)
        {
            telemetry.addData("claw not found in config file", 0);
            claw = null;
        }
        try
        {
            wrist = hardwareMap.get(Servo.class, "wrist");
            wrist.setDirection(Servo.Direction.FORWARD);
        }
        catch (Exception p_exeception)
        {
            telemetry.addData("wrist not found in config file", 0);
            wrist = null;
        }

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
        telemetry.addData("encoder: ", encoder);
        double gain = 1;
        double liftEncoderPos = encoder;

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

    public void verticalDrive(double power)
    {
        telemetry.addData("velocity: ", leftVertical.getVelocity());

        if ( encoder > 4700 && power < 0 ) {
            leftVertical.setPower(0.0);
            rightVertical.setPower(0.0);
        }
        else if(touchR.isPressed() || touchL.isPressed())
        {
            leftVertical.setPower(0.25);
            rightVertical.setPower(0.25);
        }
        else if ( encoder < 700  && power > 0)
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

    public void horizontalDrive(double position)
    {
        if(horizontal != null)
        {
            telemetry.addData("horizontal: ", horizontal.getPosition());
            horizontal.setPosition(position);
        }
    }

    public void grabClaw()
    {
        if(claw != null)
        {
            claw.setPosition(.8);
        }
    }
    public void releaseClaw()
    {
        if(claw != null)
        {
            claw.setPosition(.44);
        }
    }

    public void wristDeploy()
    {
        if(wrist != null)
        {
//            wrist.setPosition(.9);
            wrist.setPosition(.45);
        }

    }
    public void wristRetract()
    {
        if(wrist != null)
        {
            wrist.setPosition(.05);
        }
    }
    public void wristDrive( double position )
    {
        if(horizontal != null)
        {
            wrist.setPosition(position);
        }
    }

    public void goHome()
    {

        resetLiftStartTime();

        switch (state)
        {
            case 0:
                horizontalDrive(HOME);
                wristRetract();
                releaseClaw();
                verticalDrive(0.3);
                if(getLiftRuntime() > 1.25)
                {
                    resetLiftStartTime();
                    state++;
                }
                else if(touchL.isPressed() || touchR.isPressed())
                {
                    resetLiftStartTime();
                    verticalDrive(0.0);
                    state = 2;
                }
                break;

            case 1:
                horizontalDrive(HOME);
                wristRetract();
                releaseClaw();
                verticalDrive(1.0);
                if(touchL.isPressed() || touchR.isPressed())
                {
                    resetLiftStartTime();
                    verticalDrive(0.0);
                    state++;
                }
                break;

            default:
                break;

        }
    }

    public void autoExtend()
    {
        resetLiftStartTime();

        switch (stateTwo)
        {
            case 0:
                verticalDrive(-0.2);
                if (leftVertical.getCurrentPosition() >= 150)
                {
                    verticalDrive(0.0);
                    resetLiftStartTime();
                    stateTwo++;
                }
                break;

            case 1:
                horizontalDrive(EXTEND);
                if (getLiftRuntime() > 1.5)
                {
                    wristDeploy();
                    verticalDrive(.2);
                    if (getLiftRuntime() > 1.7)
                    {
                        verticalDrive(0.0);
                    }
                    resetLiftStartTime();
                    stateTwo++;
                }
                break;

            default:
                break;

        }
    }

    /**
     * Get the number of seconds this op mode has been running
     * <p>
     * This method has sub millisecond accuracy.
     * @return number of seconds this op mode has been running
     */
    public double getLiftRuntime() {
        final double NANOSECONDS_PER_SECOND = TimeUnit.SECONDS.toNanos(1);
        return (System.nanoTime() - startTime) / NANOSECONDS_PER_SECOND;
    }

    /**
     * Reset the start time to zero.
     */
    public void resetLiftStartTime() {
        startTime = System.nanoTime();
    }

    public void stopLift()
    {
        rightVertical.setPower(0.0);
        leftVertical.setPower(0.0);
    }


}
