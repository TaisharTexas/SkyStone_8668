package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.sbfUtil.DataLogger;

@TeleOp(name="teleop", group="pure")

public class Teleop extends OpMode
{
    Vehicle robot = new Vehicle(0,0, telemetry);

    private DcMotorEx RF = null;
    private DcMotorEx RR = null;
    private DcMotorEx LF = null;
    private DcMotorEx LR = null;

    private DcMotorEx xEncoder = null;
    private DcMotorEx yEncoder = null;

    private BNO055IMU gyro;

    final double encoderWheelRadius = 1.5; //in inches
    final double tickPerRotation = 2400;
    final double inchesPerRotation = 3 * Math.PI;

    int currentXEncoder = 0;
    int currentYEncoder = 0;
    double currentHeading = 0;

    double xPrev = 0;
    double yPrev = 0;
    double loopTime=0;

    private DataLogger myData;


    public void init()
    {

        RF  = hardwareMap.get(DcMotorEx.class, "rf");
        RF.setDirection(DcMotorEx.Direction.FORWARD);

        RR  = hardwareMap.get(DcMotorEx.class, "rr");
        RR.setDirection(DcMotorEx.Direction.FORWARD);

        LF  = hardwareMap.get(DcMotorEx.class, "lf");
        LF.setDirection(DcMotorEx.Direction.FORWARD);

        LR  = hardwareMap.get(DcMotorEx.class, "lr");
        LR.setDirection(DcMotorEx.Direction.FORWARD);

        xEncoder  = hardwareMap.get(DcMotorEx.class, "xEncoder");
        xEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        xEncoder.setDirection((DcMotorEx.Direction.FORWARD));

        yEncoder  = hardwareMap.get(DcMotorEx.class, "yEncoder");
        yEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        yEncoder.setDirection((DcMotorEx.Direction.REVERSE));

        try
        {
            // Set up the parameters with which we will use our IMU. Note that integration
            // algorithm here just reports accelerations to the logcat log; it doesn't actually
            // provide positional information.
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            // Retrieve and initialize the IMU.
            gyro = hardwareMap.get(BNO055IMU.class, "imu");
            gyro.initialize(parameters);
        }
        catch (Exception p_exeception)
        {
            telemetry.addData("imu not found in config file", 0);
            gyro = null;
        }

        myData = new DataLogger("8668_Robot_Data");
        myData.addField("elapsedTime");
        myData.addField("xEncoderPos");
        myData.addField("yEncoderPos");
        myData.addField("xEncoderVel");
        myData.addField("yEncoderVel");
        myData.addField("xVelCalc");
        myData.addField("yVelCalc");
        myData.newLine();

    }

    @Override
    public void init_loop()
    {

        if ( xEncoder != null )
        {
            telemetry.addData("x encoder: ", xEncoder.getCurrentPosition());
        }
        if ( yEncoder != null )
        {
            telemetry.addData("y encoder: ", yEncoder.getCurrentPosition());
        }
        if ( gyro != null )
        {
            telemetry.addData("heading: ", getHeading());
        }

        if ( xEncoder != null && yEncoder != null )
        {
            getYLinearVelocity();
        }
        telemetry.addData("path: ", Environment.getExternalStorageDirectory().getPath() + "/");
//
    }

    public void start()
    {
        resetStartTime();
    }

    public void loop()
    {

        /*
         * Update encoder values used to determine the robot's position and movement
         */
        currentXEncoder = xEncoder.getCurrentPosition();
        currentYEncoder = yEncoder.getCurrentPosition();
        loopTime = getRuntime();
        double xInchesMoved = getXInchesMoved();
        double yInchesMoved = getYInchesMoved();

        currentHeading = getHeading();
        telemetry.addData("mp.heading: ", currentHeading);

        /* Chassis Control */
        /** The x-axis of the left joystick on the gamepad. Used for chassis control*/
        double lStickX = -gamepad1.left_stick_x;
        /** The x-axis of the right joystick on the gamepad. Used for chassis control*/
        double rStickX = -gamepad1.right_stick_x;
        /** The y-axis of the left joystick on the gamepad. Used for chassis control*/
        double lStickY = gamepad1.left_stick_y;
        /** The y-axis of the right joystick on the gamepad. Used for chassis control*/
        double rStickY = gamepad1.right_stick_y;

        /*
         * Get location moved and update the robot's location
         */
        robot.location.set((float)(robot.location.x + xInchesMoved), (float)(robot.location.y + yInchesMoved));
        telemetry.addData("mp.global location: ", robot.location);

        /*
         * Calculate velocity manually using elapsed time and distance travelled.
         */
        double myVx = xInchesMoved/loopTime;
        double myVy = yInchesMoved/loopTime;

        robot.velocity.set((float)myVx, (float) myVy);
        telemetry.addData("mp.calculated vel: ", robot.velocity);

        /*
         * Get velocity from encoders for the purposes of comparing with calculated velocity.
         */
        PVector myVelocity = new PVector(getXLinearVelocity(), getYLinearVelocity());
        telemetry.addData("mp.encoder vel: ", myVelocity);

        /*
         * Grab the heading and tell the robot
         */
        robot.currentHeading = currentHeading;
        robot.currentAngularVelocity = getAngVelocity();

        /*
         * Reset previous encoder values and the run time for use in next loop iteration.
         */
        xPrev = currentXEncoder;
        yPrev = currentYEncoder;
        resetStartTime();

        /* Tell the robot to move */
        joystickDrive(lStickX, lStickY, rStickX, rStickY, 1);

        myData.addField(loopTime);
        myData.addField(currentXEncoder);
        myData.addField(currentYEncoder);
        myData.addField(myVelocity.x);
        myData.addField(myVelocity.y);
        myData.addField(myVx);
        myData.addField(myVy);
        myData.newLine();



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
        powerLimit = Range.clip(powerLimit, .05, afterburners());
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


        RF.setVelocity(rightFront * 15.7, AngleUnit.RADIANS);
        RR.setVelocity(rightRear * 15.7, AngleUnit.RADIANS);
        LF.setVelocity(leftFront * 15.7, AngleUnit.RADIANS);
        LR.setVelocity(leftRear * 15.7, AngleUnit.RADIANS);


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

    public void getEncoderTelem()
    {
        getXInchesMoved();
        getYInchesMoved();
        telemetry.addData("x encoder: ", xEncoder.getCurrentPosition());
        telemetry.addData("y encoder: ", yEncoder.getCurrentPosition());

    }

    public float getXInchesMoved()
    {
        double inchesX = (((currentXEncoder - xPrev) / tickPerRotation) * inchesPerRotation) * Math.cos(Math.toRadians(currentHeading)) +
                         (((currentYEncoder - yPrev) / tickPerRotation) * inchesPerRotation) * Math.cos(Math.toRadians(90-currentHeading));

//        telemetry.addData("mp.x inches moved: ", inchesX);

        return (float)inchesX;
    }

    public float getYInchesMoved()
    {
        double inchesY = ((-(currentXEncoder - xPrev) / tickPerRotation) * inchesPerRotation) * Math.sin(Math.toRadians(currentHeading)) +
                          (((currentYEncoder - yPrev) / tickPerRotation) * inchesPerRotation) * Math.sin(Math.toRadians(90-currentHeading));

//        telemetry.addData("mp.y inches moved: ", inchesY);

        return (float)inchesY;
    }

    public float getXLinearVelocity()
    {
        double xTicksPerSecond = xEncoder.getVelocity(AngleUnit.RADIANS) * xEncoder.getMotorType().getTicksPerRev() / 2.0 / Math.PI;

        double yTicksPerSecond =  yEncoder.getVelocity(AngleUnit.RADIANS) * yEncoder.getMotorType().getTicksPerRev() / 2.0 / Math.PI;

        double linearX = (xTicksPerSecond / tickPerRotation * 2.0 * Math.PI * encoderWheelRadius) * Math.cos(Math.toRadians(currentHeading)) +
                         (yTicksPerSecond / tickPerRotation * 2.0 * Math.PI * encoderWheelRadius) * Math.cos(Math.toRadians(90-currentHeading));

//        telemetry.addData("mp.x linear velocity: ", linearX);

        return (float)linearX*(float)0.76;
    }

    public float getYLinearVelocity()
    {
        double xTicksPerSecond = xEncoder.getVelocity(AngleUnit.RADIANS) * xEncoder.getMotorType().getTicksPerRev() / 2.0 / Math.PI;
        double yTicksPerSecond =  yEncoder.getVelocity(AngleUnit.RADIANS) * yEncoder.getMotorType().getTicksPerRev() / 2.0 / Math.PI;

        double motTicksPerSecond = RF.getVelocity(AngleUnit.RADIANS) * RF.getMotorType().getTicksPerRev() / 2.0 / Math.PI;
        telemetry.addData("encoder ticks per sec: ", yTicksPerSecond);
        telemetry.addData("motor ticks per second: ", motTicksPerSecond);

        double encodV = yEncoder.getVelocity(AngleUnit.DEGREES);
        double motoV = RF.getVelocity(AngleUnit.DEGREES);

        telemetry.addData (" encoder deg/sec: ", encodV);
        telemetry.addData ("motor deg/sec: ", motoV);

        double linearY = (-xTicksPerSecond / tickPerRotation * 2.0 * Math.PI * encoderWheelRadius ) * Math.sin(Math.toRadians(currentHeading)) +
                         ( yTicksPerSecond / tickPerRotation * 2.0 * Math.PI * encoderWheelRadius ) * Math.sin(Math.toRadians(90-currentHeading));

//        telemetry.addData("mp.y linear velocity: ", linearY);

        return (float)linearY*(float)0.76;
    }

    /**
     * Used to get the robot's heading.
     *
     * @return  the robot's heading as an double
     */
    public double getHeading()
    {
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return -AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
    }

    public double getAngVelocity()
    {
        AngularVelocity gyroReading;
        gyroReading = gyro.getAngularVelocity();
        telemetry.addData("rotation rate: ", -gyroReading.xRotationRate);
        return -gyroReading.xRotationRate;
    }

    public double afterburners()
    {
        double maximumSpeed;

        if(gamepad1.right_bumper)
        {
            maximumSpeed = 1;
        }
        else
        {
            maximumSpeed = .5;
        }

        return maximumSpeed;
    }

    public void stop()
    {
//        RF.setPower(0.0);
//        RR.setPower(0.0);
//        LF.setPower(0.0);
//        LR.setPower(0.0);
        myData.closeDataLogger();

    }
}
