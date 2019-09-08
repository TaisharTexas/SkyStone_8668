package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name="pursuit run", group="pure")

public class MecanumPursuit extends OpMode
{
    Vehicle robot = new Vehicle((float)0.0, (float)0.0, telemetry);
    //ComputerDebugging thePublisher = new ComputerDebugging();

//    private DcMotor testEncoder;
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
    double xPrev = 0;
    double yPrev = 0;
    int currentXEncoder = 0;
    int currentYEncoder = 0;
    double currentHeading = 0;

    PVector target1 = new PVector(5,30);

    Path drivePath = new Path();


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

        getEncoderTelem();

        drivePath.addPoint(0,0,15,25);
        drivePath.addPoint(0, 40, 15, 25);
        drivePath.addPoint(40,40, 15, 0);
        drivePath.addPoint(40, 0, 15, 0);
        drivePath.addPoint(0,0,15, 0);

//        ComputerDebugging.clearLogPoints();

    }

    public void loop()
    {

        currentXEncoder = xEncoder.getCurrentPosition();
        currentYEncoder = yEncoder.getCurrentPosition();
        currentHeading = getHeading();
        telemetry.addData("mp.heading: ", currentHeading);

        robot.location.set(robot.location.x+getXInchesMoved(), robot.location.y+getYInchesMoved());
        xPrev = currentXEncoder;
        yPrev = currentYEncoder;
        telemetry.addData("mp.global location: ", robot.location);

        robot.velocity.set(getXLinearVelocity(), getYLinearVelocity());
        telemetry.addData("mp.global velocity: ", robot.velocity);
        robot.currentHeading = currentHeading;

        robot.currentHeading = currentHeading;
        robot.currentAngularVelocity = getVelocity();

        telemetry.addData("mp.currentAngularVelocity: ", robot.currentAngularVelocity);

        robot.follow(drivePath);

        updateMotors(target1);

//        thePublisher.sendRobotLocation(robot);
//        thePublisher.sendLogPoint(robot.location);
//        thePublisher.markEndOfUpdate();

    }

    public void getEncoderTelem()
    {
        getXInchesMoved();
        getYInchesMoved();
        getXLinearVelocity();
        getYLinearVelocity();
    }

    public float getXInchesMoved()
    {
        double inchesX = (((currentXEncoder - xPrev) / tickPerRotation) * inchesPerRotation) * Math.cos(Math.toRadians(currentHeading)) +
                (((currentYEncoder - yPrev) / tickPerRotation) * inchesPerRotation) * Math.sin(Math.toRadians(currentHeading));

        telemetry.addData("mp.x inches moved: ", inchesX);

        return (float)inchesX;
    }

    public float getYInchesMoved()
    {
        double inchesY = ((-(currentXEncoder - xPrev) / tickPerRotation) * inchesPerRotation) * Math.sin(Math.toRadians(currentHeading)) +
                (((currentYEncoder - yPrev) / tickPerRotation) * inchesPerRotation) * Math.cos(Math.toRadians(currentHeading));

        telemetry.addData("mp.y inches moved: ", inchesY);

        return (float)inchesY;
    }

    public float getXLinearVelocity()
    {
        double linearX = (xEncoder.getVelocity(AngleUnit.RADIANS) * encoderWheelRadius ) * Math.cos(Math.toRadians(currentHeading)) +
                (yEncoder.getVelocity(AngleUnit.RADIANS) * encoderWheelRadius ) * Math.sin(Math.toRadians(currentHeading));

        telemetry.addData("mp.x linear velocity: ", linearX);

        return (float)linearX;
    }

    public float getYLinearVelocity()
    {
        double linearY = (-xEncoder.getVelocity(AngleUnit.RADIANS) * encoderWheelRadius ) * Math.sin(Math.toRadians(currentHeading)) +
                (yEncoder.getVelocity(AngleUnit.RADIANS) * encoderWheelRadius ) * Math.cos(Math.toRadians(currentHeading));

        telemetry.addData("mp.y linear velocity: ", linearY);

        return (float)linearY;
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

    public void updateMotors(PVector targetPosition)
    {
        PVector neededVeloctiy = robot.desiredVelocity.copy();

        telemetry.addData("mp.left stick velocty: ", neededVeloctiy);
        PVector headingVector = PVector.fromAngle((float)currentHeading);
        //float rotation = 

        neededVeloctiy.rotate((float)Math.toRadians(-currentHeading));

        double x = neededVeloctiy.x / 31.5; //robot.maxSpeed; //max speed is 30
        double y = neededVeloctiy.y / 31.5; // robot.maxSpeed;
        telemetry.addData("mp.right stick angular velocity: ", robot.joystickAngularVelocity);

        double turn = -robot.joystickAngularVelocity / 343;

//        telemetry.addData("mp.desired velocity x: ", x);
//        telemetry.addData("mp.desired velocity y: ", y);

        joystickDrive(-x, -y, turn, 0, 1);
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

        RF.setVelocity(rightFront * 15.7, AngleUnit.RADIANS);
        RR.setVelocity(rightRear * 15.7, AngleUnit.RADIANS);
        LF.setVelocity(leftFront * 15.7, AngleUnit.RADIANS);
        LR.setVelocity(leftRear * 15.7, AngleUnit.RADIANS);


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



    public double getVelocity()
    {
        AngularVelocity gyroReading;
        gyroReading = gyro.getAngularVelocity();
        return -gyroReading.xRotationRate;
    }

    public void stop()
    {
//        RF.setPower(0.0);
//        RR.setPower(0.0);
//        LF.setPower(0.0);
//        LR.setPower(0.0);

    }


}
